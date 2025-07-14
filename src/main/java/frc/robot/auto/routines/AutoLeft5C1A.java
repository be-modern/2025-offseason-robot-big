package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.superstructure.SuperstructureState;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;

public class AutoLeft5C1A extends AutoRoutine {
  private static final Pose2d startPose = new Pose2d(
      new Translation2d(7.140, FieldConstants.fieldWidth - 0.50),
      Rotation2d.kZero
  );

  public AutoLeft5C1A() {
    super("Left5C1A");
  }

  private Command getCoral(boolean backoff) {
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
        ).until(() -> AutoActions.isInIntakeDangerZone() || AutoActions.hasCoralAtIndexer()),
        intake()
    );
  }

  @Override
  public Command getAutoCommand() {
    var scorePreload = sequence(
        setGoal(AimGoalSupplier.ReefFace.FarLeftTilt, false, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var scoreNearL4Right = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var scoreNearL4Left = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );


    var scoreNearL3Right = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L3),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var scoreNearL3Left = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L3),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var ending = sequence(
        takeAlgae(),
        driveToEndPoint(true),
        indicateEnd()
    );


    return sequence(
        scorePreload,
        getCoral(true),
        scoreNearL4Right,
        getCoral(false),
        scoreNearL4Left,
        getCoral(false),
        scoreNearL3Right,
        getCoral(false),
        scoreNearL3Left,
        ending
    );
  }

  @Override
  public Command getOnSelectCommand() {
    return resetOnPose(startPose);
  }

}
