package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.utils.CoralRecorder;
import lib.ironpulse.command.DecisionTree;
import lib.ironpulse.utils.Logging;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Set;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;

public class AutoLeft2C extends AutoRoutine {
  private static final Pose2d startPose = new Pose2d(
      new Translation2d(7.140, FieldConstants.fieldWidth - 0.50),
      Rotation2d.kZero
  );
  private final Timer indexTimer = new Timer();
  private int idxCoral = 0;
  private boolean hasSeenCoral = false;

  public AutoLeft2C() {
    super("Left2C");
  }

  private void reset() {
    idxCoral = 0;
    hasSeenCoral = false;
//    RobotStateRecorder.setCoralFilterRegion(AllianceFlipUtil.shouldFlip() ? null : null);
  }

  private void advanceCoralIdx() {
    Logging.info("Auto", "Advancing coral idx from %d to %d.", idxCoral, idxCoral + 1);
    idxCoral++;
  }

  private Command setGoalBasedOnIdx() {
    Logging.info("Auto", "Currently on coral idx %d.", idxCoral);
    indexTimer.reset();
    indexTimer.start();

    return switch (idxCoral) {
      case 0 -> setGoal(AimGoalSupplier.ReefFace.FarLeftTilt, false, SuperstructureState.L4);
      case 1 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L4);
      case 2 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L4);
      case 3 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L3);
      case 4 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L3);
      default -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L2);
    };
  }

  @Override
  public Command getAutoCommand() {
    var start = Commands.runOnce(this::reset);

    var scorePreload = sequence(
        print("Scoring Preload"),
        defer(this::setGoalBasedOnIdx, Set.of()),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot(),
        Commands.runOnce(this::advanceCoralIdx)
    ).finallyDo(
        () -> hasSeenCoral = false
    );

    var getCoral = print("Getting Coral").andThen(
        defer(
            () -> {
              boolean backoff = idxCoral == 1; // only back off for the first coral
              return deadline(
                  sequence(
                      deadline(
                          driveToIntakePoint(true, backoff),
                          indicate(IndicatorIO.Patterns.INTAKE)
                      ).until(AutoActions::isCoralInSight),
                      deadline(
                          chase(),
                          indicate(IndicatorIO.Patterns.ASSISTED_INTAKE)
                      ).onlyIf(AutoActions::isCoralInSight)
                  ).until(() -> AutoActions.isInIntakeDangerZone() || hasSeenCoral || AutoActions.hasCoralAtEE()),
                  intake()
              );
            }, Set.of(swerve, superstructure)
        )
    );

    var driveToBackoffPoint = sequence(
        print("Driving To Backoff Point"),
        driveToBackoffPoint(true)
    ).unless(() -> hasSeenCoral || AutoActions.hasCoralAtEE());

    var score = sequence(
        print("Scoring"),
        defer(this::setGoalBasedOnIdx, Set.of()),
        sequence(
            parallel(
                driveToSelectedTarget(),
                sequence(
                    intake().onlyIf(() -> !AutoActions.hasCoralAtEE()),
                    waitUntil(AutoActions::hasCoralAtEE),
                    prepare()
                )
            ),
            sequence(
                shoot(),
                Commands.runOnce(this::advanceCoralIdx)
            )
        )
    ).finallyDo(() -> {
      hasSeenCoral = false; // reset flag at the end of score, before next chase
    });

    var end = sequence(
        print("Ending"),
        AutoActions.takeAlgae()
    );

    var tree = new DecisionTree();
    tree.addRoot(start);
    tree.addAlwaysTrueDecision(start, scorePreload);
    tree.addAlwaysTrueDecision(scorePreload, getCoral);

    // branch 1: has coral, score
    tree.addDecision(getCoral, score, () -> hasSeenCoral || hasCoralAtEE());

    // branch 2: does not have coral, drive to backoff point and retry
    tree.addDecision(getCoral, driveToBackoffPoint, () -> !hasCoralAtEE() && !hasSeenCoral);
    tree.addDecision(driveToBackoffPoint, getCoral, () -> !hasCoralAtEE() && !hasSeenCoral);
    tree.addDecision(driveToBackoffPoint, score, () -> hasSeenCoral || hasCoralAtEE());

    // go back to get coral if not reached target count
    tree.addDecision(score, getCoral, () -> this.idxCoral < 2);

    // end if scored enough coral
    tree.addDecision(score, end, () -> this.idxCoral >= 2);

    return deadline(
        tree.toCommand(),
        Commands.run(() -> {
          System.out.println(hasSeenCoral);
          if(!hasSeenCoral && superstructure.hasIndexedCoral()) hasSeenCoral = true;
        })
    );
  }

  @Override
  public Command getOnSelectCommand() {
    return resetOnPose(startPose);
  }
}
