package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.auto.AutoActions.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class AutoRight5C1A extends AutoRoutine {
  private static final Pose2d startPose = new Pose2d(
      new Translation2d(7.140, 0.500),
      Rotation2d.kZero
  );

  public AutoRight5C1A() {
    super("Right5C1A");
  }

  @Override
  public Command getAutoCommand() {
    var scorePreload = sequence(
        setGoal(AimGoalSupplier.ReefFace.FarRightTilt, false, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var driveToDpAndIntake =  sequence(
        driveToDecisionPoint(true)
    );

    return sequence(
        scorePreload,
        driveToDpAndIntake,
        indicateEnd()
    );
  }

  @Override
  public Command getOnSelectCommand() {
    return resetOnPose(startPose);
  }
}
