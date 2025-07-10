package frc.robot.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.commands.aimSequences.ChaseCoralCommand;
import frc.robot.commands.aimSequences.ReefAimCommand;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import org.littletonrobotics.AllianceFlipUtil;

import java.util.List;

import static edu.wpi.first.units.Units.*;
import static frc.robot.commands.aimSequences.AimGoalSupplier.isInHexagonalReefDangerZone;

public class AutoActions {
  private static Swerve swerve;
  private static Superstructure superstructure;
  private static IndicatorSubsystem indicator;
  private static PhotonVisionSubsystem photon;

  private static Pose2d kLeftDecisionPoint = new Pose2d(
      new Translation2d(2.0, 6.3),
      Rotation2d.fromDegrees(135)
  );
  private static Pose2d kLeftBackoff = new Pose2d(
      new Translation2d(4.0, 6.3),
      Rotation2d.fromDegrees(180)
  );

  private static Pose2d kRightDecisionPoint = new Pose2d(
      new Translation2d(2.0, 1.5),
      Rotation2d.fromDegrees(-135)
  );
  private static Pose2d kRightBackoff = new Pose2d(
      new Translation2d(4.0, 1.5),
      Rotation2d.fromDegrees(180.0)
  );


  public static void init(Swerve swerve, Superstructure superstructure, IndicatorSubsystem indicator, PhotonVisionSubsystem photon) {
    AutoActions.swerve = swerve;
    AutoActions.superstructure = superstructure;
    AutoActions.indicator = indicator;
    AutoActions.photon = photon;
  }

  public static Command intake() {
    return superstructure.runGoal(() -> SuperstructureState.CORAL_GROUND_INTAKE);
  }

  public static Command intakeUtilComplete() {
    return superstructure
        .runGoal(AutoActions::determineIntakeState)
        .until(AutoActions::isIntakeComplete);
  }

  public static Command chase() {
    return new ChaseCoralCommand(swerve, photon);
  }

  public static Command setGoal(AimGoalSupplier.ReefFace face, boolean isRight, SuperstructureState level) {
    return Commands.runOnce(() -> {
      var dest = DestinationSupplier.getInstance();
      dest.setStateSetPoint(level);
      dest.updateBranch(isRight);
      AimGoalSupplier.setSelectedTarget(face);
    });
  }

  public static Command driveToNearestTarget() {
    return new ReefAimCommand(swerve, indicator, false);
  }

  public static Command driveToSelectedTarget() {
    return new ReefAimCommand(swerve, indicator, true);
  }

  public static PathPlannerPath generatePath(List<Pose2d> waypoints, double endVelMps) {
    PathConstraints constraints = new PathConstraints(
        4.5, 7.0,
        15.0, 30.0, 12.0
    );
    List<Waypoint> pts = PathPlannerPath.waypointsFromPoses(waypoints);
    Pose2d lastPose = waypoints.get(waypoints.size() - 1);
    GoalEndState endState = new GoalEndState(endVelMps, lastPose.getRotation());
    return new PathPlannerPath(pts, constraints, null, endState);
  }

  public static Command driveToDecisionPoint(boolean isLeft) {
    return swerve.defer(() -> {
      Pose2d current = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
      Pose2d backoff = isLeft ? kLeftBackoff : kRightBackoff;
      Pose2d decision = isLeft ? kLeftDecisionPoint : kRightDecisionPoint;

      List<Pose2d> waypoints = current.getX() > backoff.getX()
          ? List.of(current, backoff, decision)
          : List.of(current, decision);

      PathPlannerPath path = generatePath(waypoints, 1.5);
      return followPath(path);
    });
  }

  public static Command followPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        () -> RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
        swerve::getChassisSpeeds,
        (vel, ff) -> {
          swerve.runTwist(vel);
        },
        new PPHolonomicDriveController(
            new PIDConstants(3.5, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0),
            RobotConstants.LOOPER_DT
        ),
        RobotConstants.AUTO_ROBOT_CONFIG,
        AllianceFlipUtil::shouldFlip,
        swerve
    );
  }

  public static Command prepare() {
    return Commands
        .runOnce(() -> DestinationSupplier.getInstance()
            .setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING))
        .andThen(
            Commands.parallel(
                Commands.waitUntil(AutoActions::isSafeToRaise)
                    .onlyIf(() -> (DestinationSupplier
                        .getInstance().getPreState() == SuperstructureState.L4))
                    .andThen(superstructure
                        .runGoal(() -> DestinationSupplier
                            .getInstance()
                            .getPreState())
                        .until(superstructure::atGoal))));

  }

  public static Command shoot() {
    return superstructure
        .runGoal(
            () -> DestinationSupplier
                .getInstance()
                .getShootState())
        .until(() -> !superstructure.hasCoral());
  }

  public static Command takeAlgae() {
    var destinationSupplier = DestinationSupplier.getInstance();
    return Commands
        .runOnce(() -> destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING))
        .andThen(
            Commands.parallel(
                superstructure
                    .runGoal(() -> DestinationSupplier
                        .getInstance()
                        .getPreState())
                    .until(superstructure::hasAlgae)
            ),
            superstructure
                .runGoal(destinationSupplier::getPreState)
                .until(() -> !isInHexagonalReefDangerZone(
                    RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d()))
                .finallyDo(() -> System.out.println("done"))
        );
  }


  public static Command reset() {
    return SwerveCommands.resetAngle(swerve, new Rotation2d())
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            })).ignoringDisable(true);
  }

  public static Command resetOnPose(Pose2d pose) {
    var resetPose = new Pose3d(AllianceFlipUtil.apply(pose));

    return SwerveCommands.reset(swerve, resetPose)
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            }))
        .ignoringDisable(true);
  }

  public static Command resetOnPathStart(PathPlannerPath path) {
    var realPath = AllianceFlipUtil.shouldFlip() ? path.flipPath() : path;

    return SwerveCommands.reset(swerve, new Pose3d(realPath.getStartingHolonomicPose().get()))
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            }))
        .onlyIf(() -> realPath.getStartingHolonomicPose().isPresent())
        .ignoringDisable(true);
  }

  public static Command limitSwerve(
      double maxVelocityMps, double maxAccelerationMps2,
      double maxAngularVelDegps, double maxAngularAccelerationDegps2
  ) {
    return Commands.runOnce(() -> swerve.setSwerveLimit(
        SwerveLimit.builder()
            .maxLinearVelocity(MetersPerSecond.of(maxVelocityMps))
            .maxSkidAcceleration(MetersPerSecondPerSecond.of(maxAccelerationMps2))
            .maxAngularVelocity(DegreesPerSecond.of(maxAngularVelDegps))
            .maxAngularAcceleration(DegreesPerSecondPerSecond.of(maxAngularAccelerationDegps2))
            .build()
    ));
  }

  public static Command unlimitSwerve() {
    return Commands.runOnce(swerve::setSwerveLimitDefault);
  }

  public static Command indicateEnd() {
    return Commands.print("Auto Ended!");
  }

  /**
   * Helper method to check if robot is in the hexagonal reef danger zone
   *
   * @return true if robot is in danger zone
   */
  private static boolean isInReefDangerZone() {
    Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    return AimGoalSupplier.isInHexagonalReefDangerZone(pose);
  }

  /**
   * Determines the appropriate intake state based on current conditions
   *
   * @return SuperstructureState for intake operation
   */
  private static SuperstructureState determineIntakeState() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    System.out.println("Intake State Decision - HasAlgae: " + hasAlgae + ", InDangerZone: " + inDangerZone);

    // If we have algae OR we're in danger zone, use indexed intake
    // Otherwise, use ground intake for safety
    if (hasAlgae || inDangerZone) {
      return SuperstructureState.CORAL_INDEXED_INTAKE;
    } else {
      return SuperstructureState.CORAL_GROUND_INTAKE;
    }
  }

  /**
   * Determines if the intake operation is complete based on current conditions
   *
   * @return true if intake is complete
   */
  private static boolean isIntakeComplete() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    if (hasAlgae) {
      // When we have algae, we need both algae AND indexed coral
      return superstructure.hasAlgae() && superstructure.indexedCoral();
    } else if (!inDangerZone) {
      // When not in danger zone, we just need coral
      return superstructure.hasCoral();
    } else {
      // When in danger zone (without algae), we need indexed coral
      return superstructure.indexedCoral();
    }
  }


  // ----------------------------------- Helpers ------------------------------------------------
  private static boolean isSafeToRaise() {
    return DestinationSupplier.isSafeToRaise(
        RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
        DestinationSupplier.getInstance().getCurrentBranch());
  }
}
