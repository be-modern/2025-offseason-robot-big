package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoActions;
import frc.robot.utils.CoralRecorder;
import lib.ironpulse.math.obstacle.Obstacle2d;
import lib.ironpulse.rbd.TransformRecorder;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static lib.ironpulse.math.MathTools.toPose2d;

public class RobotStateRecorder extends TransformRecorder {
  private static RobotStateRecorder instance;
  private static TimeInterpolatableBuffer<Pose2d> velocityRobotBuffer;
  private static CoralRecorder recorder;
  private static Field2d field;

  private RobotStateRecorder() {
    setBufferDuration(2.0);
    velocityRobotBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
    recorder = new CoralRecorder();

    // add filed
    field = new Field2d();

    // add default transforms
    putTransform(kTransformWorldDriverStationBlue, kFrameWorld, kFrameDriverStationBlue); // static: TWorldDSB
    putTransform(kTransformWorldDriverStationRed, kFrameWorld, kFrameDriverStationRed); // static TWorldDSR
    putTransform(new Pose3d(), Seconds.of(0.0), kFrameWorld, kFrameRobot); // dynamic TWorldRobot at origin
  
    Logger.recordOutput("RobotStateRecorder/LeftLollipop", new Pose2d(
      CoralRecorder.kLeftLollipop, new Rotation2d()
    ));
    Logger.recordOutput("RobotStateRecorder/RightLollipop", new Pose2d(
      CoralRecorder.kRightLollipop, new Rotation2d()
    ));
    Logger.recordOutput("RobotStateRecorder/LeftLollipopFlipped", new Pose2d(
      CoralRecorder.kLeftLollipopFlipped, new Rotation2d()
    ));
    Logger.recordOutput("RobotStateRecorder/RightLollipopFlipped", new Pose2d(
      CoralRecorder.kRightLollipopFlipped, new Rotation2d()
    ));
    Logger.recordOutput("RobotStateRecorder/MidLollipop", new Pose2d(
      CoralRecorder.kMidLollipop, new Rotation2d()
    ));
    Logger.recordOutput("RobotStateRecorder/RMidLollipopFlipped", new Pose2d(
      CoralRecorder.kMidLollipopFlipped, new Rotation2d()
    ));
    
  }

  public static RobotStateRecorder getInstance() {
    if (instance == null) {
      instance = new RobotStateRecorder();
    }
    return instance;
  }

  public static void periodic() {
    // update recorder
    recorder.update(RobotConstants.LOOPER_DT);
    Optional<CoralRecorder.CoralInfo> mostAlignedCoral = recorder.getMostInDirectionCoral(getPoseWorldRobotCurrent().toPose2d());
    Optional<CoralRecorder.CoralInfo> nearestCoral = recorder.getNearestCoral(getPoseWorldRobotCurrent().toPose2d());

    if (mostAlignedCoral.isPresent() && nearestCoral.isPresent()) {
      Logger.recordOutput("RobotStateRecorder/mostAlignedCoral", new Pose2d(mostAlignedCoral.get().getTranslation(), new Rotation2d(0)));
      Logger.recordOutput("RobotStateRecorder/nearestCoral", new Pose2d(nearestCoral.get().getTranslation(), new Rotation2d(0)));
    }

    // visualization
    var poseWorldRobot = getPoseWorldRobotCurrent();
    var posCorals = recorder.getCoralLocations();
    var velRobot = getVelocityRobotCurrent();
    var velWorldRobot = getVelocityWorldRobotCurrent();

    field.getRobotObject().setPose(poseWorldRobot.toPose2d());
    SmartDashboard.putData("Field", field);


    // logging
    Logger.recordOutput("RobotStateRecorder/poseWorldRobot", poseWorldRobot);
    Logger.recordOutput("RobotStateRecorder/velocityRobot", velRobot);
    Logger.recordOutput("RobotStateRecorder/velocityWorldRobot", velWorldRobot);
    Logger.recordOutput("RobotStateRecorder/corals", posCorals);

    boolean isInIntakeDangerZone = AutoActions.isInIntakeDangerZone();
    Logger.recordOutput("RobotStateRecorder/isInIntakeDangerZone", isInIntakeDangerZone);
    SmartDashboard.putBoolean("RobotStateRecorder/IsInIntakeDangerZone", isInIntakeDangerZone);
  }

  public static void putVelocityRobot(Time time, ChassisSpeeds speed) {
    velocityRobotBuffer.addSample(time.in(Seconds), toPose2d(speed));
  }

  public static Pose2d getVelocityRobotCurrent() {
    return velocityRobotBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
  }

  public static Pose2d getVelocityWorldRobotCurrent() {
    // robot-relative velocity (dx, dy, dθ) and current robot pose in world
    Pose2d velocityRobot = getVelocityRobotCurrent();
    Pose3d poseWorldRobot = getPoseWorldRobotCurrent();

    // drop to 2D to get the robot's heading in the XY plane
    Pose2d pose2dWR = poseWorldRobot.toPose2d();
    Translation2d velRobotTrans = velocityRobot.getTranslation();

    // rotate the translational velocity by the robot’s heading
    Translation2d velWorldTrans = velRobotTrans.rotateBy(pose2dWR.getRotation());

    // preserve the same angular component
    return new Pose2d(velWorldTrans, velocityRobot.getRotation());
  }

  public static Pose3d getPoseWorldRobotCurrent() {
    return RobotStateRecorder.getInstance().getTransform(
        Seconds.of(Timer.getTimestamp()),
        TransformRecorder.kFrameWorld,
        TransformRecorder.kFrameRobot
    ).orElse(new Pose3d());
  }

  public static Pose3d getPoseDriverRobotCurrent() {
    return RobotStateRecorder.getInstance().getTransform(
        Seconds.of(Timer.getTimestamp()),
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(
            DriverStation.Alliance.Blue) ? RobotStateRecorder.kFrameDriverStationBlue
            : RobotStateRecorder.kFrameDriverStationRed,
        TransformRecorder.kFrameRobot
    ).orElse(new Pose3d());
  }

  public static void addCoralMeasurement(Translation2d loc) {
    recorder.addCoralMeasurement(loc, RobotConstants.LOOPER_DT);
  }

  public static void setCoralFilterRegion(Obstacle2d region) {
    recorder.setFilterRegion(region);
  }

  public static Optional<CoralRecorder.CoralInfo> getNearestCoral() {
    return recorder.getNearestCoral(getPoseWorldRobotCurrent().toPose2d());
  }

  public static Optional<CoralRecorder.CoralInfo> getMostInDirectionCoral() {
    return recorder.getMostInDirectionCoral(getPoseWorldRobotCurrent().toPose2d());
  }

  public static Optional<CoralRecorder.CoralInfo> getNearestCoralInSight() {
    return recorder.getNearestCoralInSight(getPoseWorldRobotCurrent().toPose2d(), Degrees.of(70));
  }

  public static Optional<CoralRecorder.CoralInfo> getCoralById(int id) {
    return recorder.getCoralById(id);
  }
}
