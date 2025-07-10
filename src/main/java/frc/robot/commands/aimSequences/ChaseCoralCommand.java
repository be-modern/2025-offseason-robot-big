package frc.robot.commands.aimSequences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import frc.robot.utils.CoralRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.Logging;
import lib.ironpulse.utils.TimeDelayedBoolean;
import lib.ntext.NTParameter;

public class ChaseCoralCommand extends Command {
  private final Swerve swerve;
  private final PhotonVisionSubsystem vision;

  private final PIDController driveController;
  private final PIDController turnController;
  private final TimeDelayedBoolean isBlind = new TimeDelayedBoolean(0.5);
  State state = State.ACTIVE_CHASING;
  private double forwardVel = 0.0;
  private double turnVel = 0.0;
  private Rotation2d prevDirection = Rotation2d.kZero;

  private CoralRecorder.CoralInfo info = null;
  private Pose2d poseWorldRobot;

  public ChaseCoralCommand(Swerve swerve, PhotonVisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;

    driveController = new PIDController(
        ChaseCoralCommandParamsNT.driveKp.getValue(),
        ChaseCoralCommandParamsNT.driveKi.getValue(),
        ChaseCoralCommandParamsNT.driveKd.getValue());
    turnController = new PIDController(
        ChaseCoralCommandParamsNT.turnKp.getValue(),
        ChaseCoralCommandParamsNT.turnKi.getValue(),
        ChaseCoralCommandParamsNT.turnKd.getValue());

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    driveController.setP(ChaseCoralCommandParamsNT.driveKp.getValue());
    driveController.setI(ChaseCoralCommandParamsNT.driveKi.getValue());
    driveController.setD(ChaseCoralCommandParamsNT.driveKd.getValue());

    turnController.setP(ChaseCoralCommandParamsNT.turnKp.getValue());
    turnController.setI(ChaseCoralCommandParamsNT.turnKi.getValue());
    turnController.setD(ChaseCoralCommandParamsNT.turnKd.getValue());
    turnController.enableContinuousInput(0, Math.PI * 2.0);

    prevDirection = RobotStateRecorder.getPoseDriverRobotCurrent().getRotation().toRotation2d();
    driveController.reset();
    turnController.reset();
    state = State.ACTIVE_CHASING;
  }

  @Override
  public void execute() {
    // handle state transition
    RobotStateRecorder.getNearestCoral().ifPresentOrElse(info -> {
      state = State.ACTIVE_CHASING;
      this.info = info;
    }, () -> {
      state = State.BLIND_CHASING;
    });

    // get
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();

    // run state
    switch (state) {
      case ACTIVE_CHASING -> {
        Logging.info("Commands/ChaseCoralCommand", "Active Chasing!");
        Translation2d vecRobotTarget = info.translation.minus(poseWorldRobot.getTranslation());
        prevDirection = vecRobotTarget.getAngle();

        forwardVel = -driveController.calculate(
            vecRobotTarget.getNorm(),
            0.0
        );
        forwardVel = MathUtil.clamp(
            forwardVel,
            0.0, ChaseCoralCommandParamsNT.activeChaseMaxVelocityMps.getValue()
        );

        turnVel = turnController.calculate(
            poseWorldRobot.getRotation().getRadians(),
            prevDirection.getRadians()
        );
      }

      case BLIND_CHASING -> {
        Logging.info("Commands/ChaseCoralCommand", "Blind Chasing!");
        forwardVel = MathUtil.clamp(
            forwardVel, 0.0, ChaseCoralCommandParamsNT.blindChaseMaxVelocityMps.getValue()
        );
        turnVel = 0.0;
      }
    }

    // run target
    Translation2d velWorld = new Translation2d(forwardVel, prevDirection);
    swerve.runTwist(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velWorld.getX(),
            velWorld.getY(),
            turnVel,
            poseWorldRobot.getRotation()
        )
    );

  }

  @Override
  public void end(boolean interrupted) {
    swerve.runStop();
  }

  @Override
  public boolean isFinished() {
    return isBlind.update(state == State.BLIND_CHASING, ChaseCoralCommandParamsNT.blindChaseMaxTimeSeconds.getValue());
  }

  private enum State {
    ACTIVE_CHASING, BLIND_CHASING
  }

  @NTParameter(tableName = "Params/Commands/ChaseCoralCommand")
  public static class ChaseCoralCommandParams {
    static final double driveKp = 3.5;
    static final double driveKi = 0.0;
    static final double driveKd = 0.1;

    static final double turnKp = 4.0;
    static final double turnKi = 0.0;
    static final double turnKd = 0.3;

    static final double activeChaseMaxVelocityMps = 2.0;
    static final double blindChaseMaxTimeSeconds = 0.5;
    static final double blindChaseMaxVelocityMps = 1.5;
  }
}
