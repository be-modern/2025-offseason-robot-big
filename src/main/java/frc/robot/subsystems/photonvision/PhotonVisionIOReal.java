package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

import static frc.robot.RobotConstants.PhotonvisionConstants.*;
import org.littletonrobotics.junction.Logger;

public class PhotonVisionIOReal implements PhotonVisionIO {

    private final String name;
    private final PhotonCamera camera;
    private final int id;
    private double lastObservedConf;
    private Pose2d nearestCoralPosition = null;
    private int lastObservedPeriod = 0;

    public PhotonVisionIOReal(int id) {
        this.id = id;
        this.name = PV_CAMERA_NAMES[id];
        System.out.println("PhotonVision: Attempting to connect to camera: " + name);
        camera = new PhotonCamera(name);
        System.out.println("PhotonVision: Camera instance created for: " + name);
    }

    /**
     * Uses target information provided by PhotonVision to estimate the ground position (Translation2d) of the target relative to robot.
     *
     * @param target         The target detected by PhotonVision
     * @param cameraHeight   Height of the camera from the ground (in meters)
     * @param cameraPitchRad Pitch angle of the camera relative to the ground (positive = up, in radians)
     * @param cameraToRobot  Transform2d representing the camera's position and orientation relative to the robot
     *                       //* @param robotPose        Current Pose2d of the robot on the field
     * @return Pose2d of the detected target on the field (in field coordinate system)
     */
    public static Transform2d estimateGroundTargetPose(
            PhotonTrackedTarget target,
            double cameraHeight,
            double cameraPitchRad,
            Transform2d cameraToRobot
            //Pose2d robotPose
    ) {
        // Get yaw and pitch from the target (PhotonVision provides these in degrees)
        double yawRad = Math.toRadians(target.getYaw());
        double pitchRad = Math.toRadians(target.getPitch());

        // Calculate total vertical angle from camera to target
        double totalPitch = cameraPitchRad + pitchRad;

        // Prevent division by zero or extremely small angles
        if (Math.abs(Math.tan(totalPitch)) < 1e-5) {
            return null;  // Or throw an exception
        }

        // Compute horizontal ground distance from camera to target
        double distance = cameraHeight / Math.tan(totalPitch);

        // Target position in the camera's coordinate system
        double xCamera = distance * Math.cos(yawRad);  // forward
        double yCamera = distance * Math.sin(yawRad);  // left/right

        Translation2d targetRelativeToCamera = new Translation2d(xCamera, yCamera);

        // Convert from camera-relative to robot-relative coordinates
        Transform2d cameraToTarget = new Transform2d(targetRelativeToCamera, new Rotation2d());
        Transform2d robotToTarget = cameraToRobot.plus(cameraToTarget);

        return robotToTarget;
//        // Convert from robot-relative to field-relative coordinates
//        Pose2d targetPose = robotPose.plus(robotToTarget);
//
//        return targetPose;
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        processResult();
        inputs.connected = camera.isConnected();
        inputs.name = camera.getName();
        inputs.nearestCoralPosition = nearestCoralPosition;
        inputs.id = id;
        inputs.lastObservedConf = lastObservedConf;
        inputs.lastObservedPeriod = lastObservedPeriod;
    }

    @Override
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }

    private void processResult() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            lastObservedPeriod++;
            if (lastObservedPeriod > 5) nearestCoralPosition = null;
            return;
        }
        for (int i = results.size() - 1; i >= 0; i--) {
            /*
                Logic:
                1. Find the latest non-empty result
                2. Detect the nearest relative to robot.
                3. Return the Pose.
             */
            PhotonPipelineResult result = results.get(i);
            if (!result.hasTargets()) continue;
            
            // Reset period counter since we have new targets
            lastObservedPeriod = 0;
            
            // Find the nearest target
            Transform2d nearestTargetTransform = null;
            double nearestDistance = Double.MAX_VALUE;
            double bestConfidence = 0.0;
            
            // Get camera configuration
            double cameraHeight = CAMERA_HEIGHT_METERS.get();
            double cameraPitchRad = Math.toRadians(CAMERA_PITCH_DEGREES.get());
            Transform2d cameraToRobot = new Transform2d(
                new Translation2d(CAMERA_TO_ROBOT_X.get(), CAMERA_TO_ROBOT_Y.get()),
                new Rotation2d(Math.toRadians(CAMERA_TO_ROBOT_ROTATION_DEGREES.get()))
            );
            
            for (PhotonTrackedTarget target : result.getTargets()) {
                Transform2d targetTransform = estimateGroundTargetPose(
                    target,
                    cameraHeight,
                    cameraPitchRad,
                    cameraToRobot
                );
                
                if (targetTransform != null) {
                    double distance = targetTransform.getTranslation().getNorm();
                    
                    // Prefer closer targets, but also consider confidence
                    if (distance < nearestDistance && target.getPoseAmbiguity() >= 0) {
                        nearestDistance = distance;
                        nearestTargetTransform = targetTransform;
                        bestConfidence = 1.0 - target.getPoseAmbiguity();
                    }
                }
            }
            
            // Update nearest coral position if we found a valid target
            if (nearestTargetTransform != null) {
                nearestCoralPosition = new Pose2d(
                    nearestTargetTransform.getTranslation(),
                    nearestTargetTransform.getRotation()
                );
                lastObservedConf = bestConfidence;
                
                // Log the detected coral position for debugging
                Logger.recordOutput("PhotonVision/Camera" + id + "/NearestCoralDistance", nearestDistance);
                Logger.recordOutput("PhotonVision/Camera" + id + "/NearestCoralConfidence", bestConfidence);
                
                System.out.println("PhotonVision Camera " + id + " detected coral at distance: " + nearestDistance + " meters");
                break; // Use the latest result with targets
            }
        }
    }
}
