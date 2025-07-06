package frc.robot.commands.aimSequences;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.commands.SwerveDriveToPoseParamsNT;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class ReefAimCommand extends Command {
  private final static String kTag = "Commands/ReefAimCommand";
  private final Swerve swerve;
  private final IndicatorSubsystem indicatorSubsystem;
  private boolean rightReef; // true if shooting right reef
  private boolean translationOnTarget = false;
  private boolean rotationOnTarget = false;
  private boolean translationStationary = false;
  private boolean rotationStationary = false;
  private Pose2d poseWorldRobot, velocityWorldRobot, tagPose, poseWorldTarget, finalDestinationPose;
  private Translation2d translationalVelocity, controllerVelocity;
  private ProfiledPIDController translationController;
  private ProfiledPIDController rotationController;


  public ReefAimCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem) {
    this.indicatorSubsystem = indicatorSubsystem;
    this.swerve = swerve;

    translationController = new ProfiledPIDController(
        ReefAimCommandParamsNT.translationKp.getValue(),
        ReefAimCommandParamsNT.translationKi.getValue(),
        ReefAimCommandParamsNT.translationKd.getValue(),
        new TrapezoidProfile.Constraints(
            ReefAimCommandParamsNT.translationVelocityMax.getValue(),
            ReefAimCommandParamsNT.translationAccelerationMax.getValue()
        )
    );
    rotationController = new ProfiledPIDController(
        ReefAimCommandParamsNT.rotationKp.getValue(),
        ReefAimCommandParamsNT.rotationKi.getValue(),
        ReefAimCommandParamsNT.rotationKd.getValue(),
        new TrapezoidProfile.Constraints(
            ReefAimCommandParamsNT.rotationVelocityMax.getValue(),
            ReefAimCommandParamsNT.rotationAccelerationMax.getValue()
        )
    );
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // tuning
    if (RobotConstants.TUNING) {
      translationController.setP(ReefAimCommandParamsNT.translationKp.getValue());
      translationController.setI(SwerveDriveToPoseParamsNT.translationKi.getValue());
      translationController.setD(SwerveDriveToPoseParamsNT.translationKd.getValue());
      translationController.setConstraints(new TrapezoidProfile.Constraints(
          SwerveDriveToPoseParamsNT.translationVelocityMax.getValue(),
          SwerveDriveToPoseParamsNT.rotationAccelerationMax.getValue()
      ));

      rotationController.setP(SwerveDriveToPoseParamsNT.rotationKp.getValue());
      rotationController.setI(SwerveDriveToPoseParamsNT.rotationKi.getValue());
      rotationController.setD(SwerveDriveToPoseParamsNT.rotationKd.getValue());
      rotationController.setConstraints(new TrapezoidProfile.Constraints(
          SwerveDriveToPoseParamsNT.rotationVelocityMax.getValue(),
          SwerveDriveToPoseParamsNT.rotationAccelerationMax.getValue()
      ));
    }

    // get current state
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    velocityWorldRobot = RobotStateRecorder.getVelocityWorldRobotCurrent();

    // calculate destination
    tagPose = AimGoalSupplier.getNearestTag(poseWorldRobot);

    // choose target based on game piece
    if (DestinationSupplier.getInstance().getCurrentGamePiece() == DestinationSupplier.GamePiece.ALGAE_INTAKING) {
      finalDestinationPose = AimGoalSupplier.getFinalAlgaeTarget(tagPose);
    } else {
      rightReef = DestinationSupplier.getInstance().getCurrentBranch();
      finalDestinationPose = AimGoalSupplier.getFinalCoralTarget(tagPose, rightReef);
    }

    // Now that finalDestinationPose is set, we can get the drive target
    poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);

    // PID init with field-relative velocities
    translationController.reset(velocityWorldRobot.getTranslation().getNorm());
    rotationController.reset(velocityWorldRobot.getRotation().getRadians());
    indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
  }

  @Override
  public void execute() {
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
    velocityWorldRobot = RobotStateRecorder.getVelocityWorldRobotCurrent();

    // compute translation error, tu
    Translation2d pRT = poseRobotTarget.getTranslation();
    double pRT_norm = pRT.getNorm();
    Rotation2d pRT_dir = pRT.getAngle();
    // NOTE: as pRT_norm is always positive, then vRT_norm is always negative.
    // to make the robot move along but not opposite to pRT_dir, we take the minus sign before vRT_norm
    double vRT_norm = translationController.calculate(pRT_norm, 0.0);
    Translation2d vRT = new Translation2d(-vRT_norm, pRT_dir);

    // compute rotation err, turn into angular velocity scalar
    double thetaRT = poseRobotTarget.getRotation().getRadians();
    double omegaRT = -rotationController.calculate(thetaRT, 0.0);

    // compose and run velocity
    ChassisSpeeds VRT = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
    swerve.runTwist(VRT);

    // logging
    Logger.recordOutput(kTag + "/tagPose", tagPose);
    Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
    Logger.recordOutput(kTag + "/finalDestinationPose", finalDestinationPose);
    Logger.recordOutput(kTag + "/translationalVelocity", translationalVelocity);
    Logger.recordOutput(kTag + "/controllerVelocity", controllerVelocity);
  }

  @Override
  public boolean isFinished() {
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
    translationOnTarget = epsilonEquals(
        poseRobotTarget.getTranslation(), new Translation2d(),
        ReefAimCommandParamsNT.translationOnTargetToleranceMeter.getValue()
    );
    rotationOnTarget = epsilonEquals(
        poseRobotTarget.getRotation().getDegrees(),
        0.0,
        ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue()
    );

    translationStationary = epsilonEquals(
        velocityWorldRobot.getTranslation(), new Translation2d(),
        ReefAimCommandParamsNT.translationOnTargetVelocityMetersPerSecond.getValue()
    );
    rotationStationary = epsilonEquals(
        velocityWorldRobot.getRotation().getDegrees(), 0.0,
        ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue()
    );

    Logger.recordOutput(kTag + "/translationOnTarget", translationOnTarget);
    Logger.recordOutput(kTag + "/rotationOnTarget", rotationOnTarget);
    Logger.recordOutput(kTag + "/translationStationary", translationStationary);
    Logger.recordOutput(kTag + "/rotationStationary", rotationStationary);
    return translationOnTarget && rotationOnTarget && translationStationary && rotationStationary;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.runStop();
    if (!interrupted) indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
    else indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);

  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @NTParameter(tableName = "Params/" + kTag)
  public static class ReefAimCommandParams {
    static final double translationKp = 4.8;
    static final double translationKi = 0.05;
    static final double translationKd = 0.2;
    static final double translationVelocityMax = 4.2;
    static final double translationAccelerationMax = 25.0;

    static final double rotationKp = 5.0;
    static final double rotationKi = 0.0;
    static final double rotationKd = 0.0;
    static final double rotationVelocityMax = 7.0;
    static final double rotationAccelerationMax = 20.0;

    static final double translationOnTargetToleranceMeter = 0.01;
    static final double translationOnTargetVelocityMetersPerSecond = 0.05;
    static final double rotationOnTargetToleranceDegree = 1;
    static final double rotationOnTargetVelocityToleranceDegreesPerSecond = 5.0;
  }
}