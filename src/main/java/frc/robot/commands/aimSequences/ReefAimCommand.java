package frc.robot.commands.aimSequences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.Superstructure;
import lib.ironpulse.math.MathTools;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.Logging;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.*;

public class ReefAimCommand extends Command {
  private final static String kTag = "Commands/ReefAimCommand";
  private final Swerve swerve;
  private final IndicatorSubsystem indicatorSubsystem;
  private boolean rightReef; // true if shooting right reef
  private boolean xOnTarget = false;
  private boolean xStationary = false;
  private boolean yOnTarget = false;
  private boolean yStationary = false;
  private boolean rotationOnTarget = false;
  private boolean rotationStationary = false;
  private boolean imuStable = false;
  private Pose2d poseWorldRobot, velocityRobot, tagPose, poseWorldTarget, finalDestinationPose;
  private ProfiledPIDController xController;
  private PIDController yController;
  private PIDController rotationController;

  private final boolean useSelectedTarget;

  public ReefAimCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem, boolean useSelectedTarget) {
    this.indicatorSubsystem = indicatorSubsystem;
    this.swerve = swerve;

    xController = new ProfiledPIDController(
        ReefAimCommandParamsNT.xKp.getValue(),
        ReefAimCommandParamsNT.xKi.getValue(),
        ReefAimCommandParamsNT.xKd.getValue(),
        new TrapezoidProfile.Constraints(
            ReefAimCommandParamsNT.xVelMax.getValue(),
            ReefAimCommandParamsNT.xAccMax.getValue()
        )
    );
    yController = new PIDController(
        ReefAimCommandParamsNT.yKp.getValue(),
        ReefAimCommandParamsNT.yKi.getValue(),
        ReefAimCommandParamsNT.yKd.getValue()
    );
    rotationController = new PIDController(
        ReefAimCommandParamsNT.rotationKp.getValue(),
        ReefAimCommandParamsNT.rotationKi.getValue(),
        ReefAimCommandParamsNT.rotationKd.getValue()
    );
    addRequirements(swerve);

    this.useSelectedTarget = useSelectedTarget;
  }

  public ReefAimCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem) {
    this(swerve, indicatorSubsystem, false);
  }

  @Override
  public void initialize() {
    // tuning
    if (RobotConstants.TUNING) {
      xController.setP(ReefAimCommandParamsNT.xKp.getValue());
      xController.setI(ReefAimCommandParamsNT.xKi.getValue());
      xController.setIZone(ReefAimCommandParamsNT.xKiZone.getValue());
      xController.setD(ReefAimCommandParamsNT.xKd.getValue());
      xController.setConstraints(
          new TrapezoidProfile.Constraints(
              ReefAimCommandParamsNT.xVelMax.getValue(),
              ReefAimCommandParamsNT.xAccMax.getValue()
          )
      );

      yController.setP(ReefAimCommandParamsNT.yKp.getValue());
      yController.setI(ReefAimCommandParamsNT.yKi.getValue());
      yController.setIZone(ReefAimCommandParamsNT.yKiZone.getValue());
      yController.setD(ReefAimCommandParamsNT.yKd.getValue());

      rotationController.setP(ReefAimCommandParamsNT.rotationKp.getValue());
      rotationController.setI(ReefAimCommandParamsNT.rotationKi.getValue());
      rotationController.setIZone(ReefAimCommandParamsNT.rotationKiZone.getValue());
      rotationController.setD(ReefAimCommandParamsNT.rotationKd.getValue());
      rotationController.setTolerance(
          ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue() / 180.0f * Math.PI,
          ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue() / 180.0f * Math.PI
      );
    }

    // get current state
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    velocityRobot = RobotStateRecorder.getVelocityRobotCurrent();

    // calculate destination
    tagPose = useSelectedTarget ? AimGoalSupplier.getSelectedTag() : AimGoalSupplier.getNearestTag(poseWorldRobot);

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
    rotationController.enableContinuousInput(0, Math.PI * 2);
    xController.reset(velocityRobot.getTranslation().getX());
    yController.reset();
    rotationController.reset();
    indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
  }

  @Override
  public void execute() {
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
    velocityRobot = RobotStateRecorder.getVelocityRobotCurrent();

    // compute translation error, tu
    Translation2d pRT = poseRobotTarget.getTranslation();
    double vxRT = -xController.calculate(pRT.getX(), 0.0);
    double vyRT = -yController.calculate(pRT.getY(), 0.0);

    // compute rotation err, turn into angular velocity scalar
    double thetaRTOriginal = poseRobotTarget.getRotation().getRadians();
    double thetaRTAdjusted = poseRobotTarget.getTranslation().getAngle().getRadians();
    double maxThetaAdjustment = degreesToRadians(ReefAimCommandParamsNT.rotationAdjustmentMaxDegree.getValue());
    thetaRTAdjusted = MathUtil.clamp(thetaRTAdjusted, thetaRTOriginal - maxThetaAdjustment, thetaRTOriginal + maxThetaAdjustment);
    double omegaRT = -rotationController.calculate(thetaRTAdjusted, 0.0);

    // set limit
    double dCurr = finalDestinationPose.relativeTo(poseWorldRobot).getTranslation().getNorm(); // use final destination
    double vFar = ReefAimCommandParamsNT.translationVelocityMaxFar.getValue();
    double vNear = ReefAimCommandParamsNT.translationVelocityMaxNear.getValue();
    double dChange = ReefAimCommandParamsNT.translationParamsChangeDistance.getValue();
    double maxTranslationVelocityMps = dCurr > dChange ? vFar : vNear + dCurr / dChange * (vFar - vNear);
    Translation2d vRT = new Translation2d(vxRT, vyRT);

    // compose and run velocity with limit
    swerve.setSwerveLimit(
        SwerveLimit.builder()
            .maxLinearVelocity(MetersPerSecond.of(maxTranslationVelocityMps))
            .maxSkidAcceleration(MetersPerSecondPerSecond.of(ReefAimCommandParamsNT.translationAccelerationMax.getValue()))
            .maxAngularVelocity(DegreesPerSecond.of(ReefAimCommandParamsNT.rotationVelocityMax.getValue()))
            .maxAngularAcceleration(DegreesPerSecondPerSecond.of(ReefAimCommandParamsNT.rotationAccelerationMax.getValue()))
            .build()
    );
    ChassisSpeeds VRT = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
    swerve.runTwist(VRT);

    // logging
    Logger.recordOutput(kTag + "/tagPose", tagPose);
    Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
    Logger.recordOutput(kTag + "/finalDestinationPose", finalDestinationPose);
    Logger.recordOutput(kTag + "/maxTranslationVelocityMps", maxTranslationVelocityMps);
  }

  @Override
  public boolean isFinished() {
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
    xOnTarget = epsilonEquals(
        poseRobotTarget.getTranslation().getX(), 0.0,
        ReefAimCommandParamsNT.xOnTargetMeter.getValue()
    );
    yOnTarget = epsilonEquals(
        poseRobotTarget.getTranslation().getY(), 0.0,
        ReefAimCommandParamsNT.yOnTargetMeter.getValue()
    );
    rotationOnTarget = epsilonEquals(
        poseRobotTarget.getRotation().getDegrees(),
        0.0,
        ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue()
    );

    xStationary = epsilonEquals(
        velocityRobot.getTranslation().getX(), 0.0,
        ReefAimCommandParamsNT.xStationaryMps.getValue()
    );
    yStationary = epsilonEquals(
        velocityRobot.getTranslation().getY(), 0.0,
        ReefAimCommandParamsNT.yStationaryMps.getValue()
    );
    rotationStationary = epsilonEquals(
        velocityRobot.getRotation().getDegrees(), 0.0,
        ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue()
    );

    imuStable = epsilonEquals(
        swerve.getEstimatedPose().getRotation().getY(), -0.01, 0.025
    );

    Logger.recordOutput(kTag + "/xOnTarget", xOnTarget);
    Logger.recordOutput(kTag + "/yOnTarget", yOnTarget);
    Logger.recordOutput(kTag + "/rotationOnTarget", rotationOnTarget);
    Logger.recordOutput(kTag + "/xStationary", xStationary);
    Logger.recordOutput(kTag + "/yStationary", yStationary);
    Logger.recordOutput(kTag + "/rotationStationary", rotationStationary);
    Logger.recordOutput(kTag + "/imuStable", imuStable);
    return (xOnTarget && yOnTarget && rotationOnTarget && xStationary && yStationary && rotationStationary && imuStable);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setSwerveLimitDefault();
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
    static final double xKp = 3.7;
    static final double xKi = 0.0;
    static final double xKiZone = 0.0;
    static final double xKd = 0.1;
    static final double xVelMax = 4.0;
    static final double xAccMax = 3.5;

    static final double yKp = 4.7;
    static final double yKi = 0.0;
    static final double yKiZone = 0.00;
    static final double yKd = 0.5;

    static final double translationVelocityMaxFar = 4.6;
    static final double translationVelocityMaxNear = 3.5;
    static final double translationParamsChangeDistance = 2.2;
    static final double translationAccelerationMax = 10.0;

    static final double rotationKp = 5.0;
    static final double rotationKi = 0.0;
    static final double rotationKiZone = 0.0;
    static final double rotationKd = 0.2;
    static final double rotationVelocityMax = 550.0;
    static final double rotationAccelerationMax = 2500.0;

    static final double xOnTargetMeter = 0.05;
    static final double yOnTargetMeter = 0.02;
    static final double xStationaryMps = 0.35;
    static final double yStationaryMps = 0.10;
    static final double rotationOnTargetToleranceDegree = 2.0;
    static final double rotationOnTargetVelocityToleranceDegreesPerSecond = 20.0;
    static final double rotationAdjustmentMaxDegree = 0.0;
  }
}