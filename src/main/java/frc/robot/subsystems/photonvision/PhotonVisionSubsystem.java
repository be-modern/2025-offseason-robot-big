package frc.robot.subsystems.photonvision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.RobotConstants.PhotonvisionConstants.SNAPSHOT_ENABLED;
import static frc.robot.RobotConstants.PhotonvisionConstants.SNAPSHOT_PERIOD;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonVisionIO[] ios;
    private final PhotonVisionIOInputsAutoLogged[] inputs;
    private Timer snapshotTimer = new Timer();

    public PhotonVisionSubsystem(PhotonVisionIO... ios) {
        this.ios = ios;
        inputs = new PhotonVisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PhotonVisionIOInputsAutoLogged();
        }
        snapshotTimer.start();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("PhotonVision/Inst" + i, inputs[i]);
        }
        if (snapshotTimer.hasElapsed(SNAPSHOT_PERIOD)) {
            for (int i = 0; i < ios.length; i++) {
                if (SNAPSHOT_ENABLED[i]) ios[i].takeOutputSnapshot();
            }
            snapshotTimer.reset();
        }
        
        // Perform 3D projection for all detected targets
        performPeriodicProjections();
    }
    
    /**
     * Performs 3D projections for all detected targets in periodic()
     * This runs the basic trigonometry-based projection method for continuous logging
     * Uses PhotonVision's getAllUnreadResults() API to only process fresh vision data,
     * preventing objects from moving with robot when vision tracking is lost
     */
    private void performPeriodicProjections() {
        // Get camera parameters from RobotConstants (tunable for calibration)
        final int CAMERA_RESOLUTION_X = (int) RobotConstants.PhotonvisionConstants.CAMERA_RESOLUTION_X.get();
        final int CAMERA_RESOLUTION_Y = (int) RobotConstants.PhotonvisionConstants.CAMERA_RESOLUTION_Y.get();
        
        // Process only fresh detections to prevent stale coordinate transformations
        List<RawDetection> freshDetections = getFreshRawDetections();
        
        for (RawDetection detection : freshDetections) {
            // Get robot-relative 3D pose
            Pose3d robotRelativePose = projectTargetTo3D(detection, CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y);
            
            // Get robot's pose from when detection was actually captured (not current pose!)
            double detectionTimeSeconds = detection.timestampMs / 1000.0;
            Pose3d robotWorldPoseAtDetection;
            try {
                robotWorldPoseAtDetection = RobotStateRecorder.getInstance().getTransform(
                    edu.wpi.first.units.Units.Seconds.of(detectionTimeSeconds),
                    lib.ironpulse.rbd.TransformRecorder.kFrameWorld,
                    lib.ironpulse.rbd.TransformRecorder.kFrameRobot
                ).orElse(RobotStateRecorder.getPoseWorldRobotCurrent()); // Fallback to current if historical not available
            } catch (Exception e) {
                // If historical pose lookup fails, use current pose as fallback
                robotWorldPoseAtDetection = RobotStateRecorder.getPoseWorldRobotCurrent();
                System.out.println("PhotonVision: Failed to get historical robot pose for detection, using current pose. Error: " + e.getMessage());
            }
            
            // Transform to world coordinates using robot pose from when detection was made
            Pose3d worldPose = robotWorldPoseAtDetection.transformBy(new Transform3d(robotRelativePose.getTranslation(), robotRelativePose.getRotation()));
            
            // Log both robot-relative and world poses
            String basePath = "PhotonVision/Camera" + detection.cameraId + "/Target" + getTargetIndex(detection);
            Logger.recordOutput(basePath + "/RobotRelativePose3D", robotRelativePose);
            Logger.recordOutput(basePath + "/WorldPose3D", worldPose);
            
            // Log world pose components separately for easier analysis
            Logger.recordOutput(basePath + "/WorldX", worldPose.getX());
            Logger.recordOutput(basePath + "/WorldY", worldPose.getY());
            Logger.recordOutput(basePath + "/WorldZ", worldPose.getZ());
            
            // Log data processing info and coordinate transform details
            Logger.recordOutput(basePath + "/ProcessedFreshData", true);
            Logger.recordOutput(basePath + "/DetectionTimestamp", detection.timestampMs);
            Logger.recordOutput(basePath + "/DetectionTimeSeconds", detectionTimeSeconds);
            Logger.recordOutput(basePath + "/RobotPoseAtDetection", robotWorldPoseAtDetection);
            
            // Log current vs historical robot pose for debugging rotation issues
            Pose3d currentRobotPose = RobotStateRecorder.getPoseWorldRobotCurrent();
            double rotationDiffDegrees = Math.toDegrees(
                currentRobotPose.getRotation().getZ() - robotWorldPoseAtDetection.getRotation().getZ()
            );
            Logger.recordOutput(basePath + "/RobotRotationDiffDegrees", rotationDiffDegrees);
        }
        
        // Log detection counts for debugging
        int totalDetections = getAllRawDetections().size();
        Logger.recordOutput("PhotonVision/TotalDetections", totalDetections);
        Logger.recordOutput("PhotonVision/FreshDetections", freshDetections.size());
        
        // Debug output to show coordinate transformation behavior
        if (freshDetections.size() > 0) {
            System.out.println("PhotonVision: Processing " + freshDetections.size() + 
                " fresh detections for coordinate transforms (total detections: " + totalDetections + 
                ") - Y-axis corrected for PhotonVision coordinate system");
        } else if (totalDetections > 0) {
            System.out.println("PhotonVision: " + totalDetections + 
                " total detections but no fresh data - maintaining last known world coordinates");
        }
    }
    
    /**
     * Gets the target index within a camera for logging purposes
     * @param detection The detection to find the index for
     * @return The target index within its camera
     */
    private int getTargetIndex(RawDetection detection) {
        List<RawDetection> cameraDetections = getDetectionsFromCamera(detection.cameraId);
        for (int i = 0; i < cameraDetections.size(); i++) {
            if (cameraDetections.get(i).equals(detection)) {
                return i;
            }
        }
        return 0; // Fallback
    }



    /**
     * Gets all raw detection data from all cameras
     * @return List of all raw detections with camera information
     */
    public List<RawDetection> getAllRawDetections() {
        List<RawDetection> allDetections = new ArrayList<>();
        
        for (int i = 0; i < inputs.length; i++) {
            PhotonVisionIOInputsAutoLogged input = inputs[i];
            if (input.hasTargets && input.targetCount > 0) {
                for (int j = 0; j < input.targetCount; j++) {
                    double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                    double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                    double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                    double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                    double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                    int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                    double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                    double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;
                    
                    allDetections.add(new RawDetection(
                        i, // camera ID
                        input.timestampMs,
                        yaw,
                        pitch,
                        area,
                        skew,
                        ambiguity,
                        fiducialId,
                        pixelX,
                        pixelY
                    ));
                }
            }
        }
        
        return allDetections;
    }

    /**
     * Gets only fresh raw detection data from all cameras (for coordinate transformations)
     * @return List of fresh raw detections with camera information
     */
    public List<RawDetection> getFreshRawDetections() {
        List<RawDetection> freshDetections = new ArrayList<>();
        
        for (int i = 0; i < inputs.length; i++) {
            PhotonVisionIOInputsAutoLogged input = inputs[i];
            // Only process detections when we have fresh data
            if (input.hasFreshData && input.hasTargets && input.targetCount > 0) {
                for (int j = 0; j < input.targetCount; j++) {
                    double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                    double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                    double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                    double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                    double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                    int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                    double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                    double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;
                    
                    freshDetections.add(new RawDetection(
                        i, // camera ID
                        input.timestampMs,
                        yaw,
                        pitch,
                        area,
                        skew,
                        ambiguity,
                        fiducialId,
                        pixelX,
                        pixelY
                    ));
                }
            }
        }
        
        return freshDetections;
    }

    /**
     * Gets raw detection data from a specific camera
     * @param cameraId The ID of the camera
     * @return List of raw detections from the specified camera
     */
    public List<RawDetection> getDetectionsFromCamera(int cameraId) {
        if (cameraId < 0 || cameraId >= inputs.length) {
            return List.of();
        }
        
        List<RawDetection> detections = new ArrayList<>();
        PhotonVisionIOInputsAutoLogged input = inputs[cameraId];
        
        if (input.hasTargets && input.targetCount > 0) {
            for (int j = 0; j < input.targetCount; j++) {
                double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;
                
                detections.add(new RawDetection(
                    cameraId,
                    input.timestampMs,
                    yaw,
                    pitch,
                    area,
                    skew,
                    ambiguity,
                    fiducialId,
                    pixelX,
                    pixelY
                ));
            }
        }
        
        return detections;
    }

    /**
     * Gets the largest target (by area) from all cameras
     * @return The largest target detection, or null if no targets
     */
    public RawDetection getLargestTarget() {
        RawDetection largest = null;
        double maxArea = 0.0;
        
        for (RawDetection detection : getAllRawDetections()) {
            if (detection.area > maxArea) {
                maxArea = detection.area;
                largest = detection;
            }
        }
        
        return largest;
    }

    /**
     * Gets the target closest to the crosshair (smallest yaw angle) from all cameras
     * @return The target closest to crosshair, or null if no targets
     */
    public RawDetection getClosestToCrosshair() {
        RawDetection closest = null;
        double minYaw = Double.MAX_VALUE;
        
        for (RawDetection detection : getAllRawDetections()) {
            double absYaw = Math.abs(detection.yaw);
            if (absYaw < minYaw) {
                minYaw = absYaw;
                closest = detection;
            }
        }
        
        return closest;
    }

    /**
     * Checks if any camera has targets
     * @return true if any camera has targets, false otherwise
     */
    public boolean hasAnyTargets() {
        for (PhotonVisionIOInputsAutoLogged input : inputs) {
            if (input.hasTargets) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the total number of targets detected across all cameras
     * @return Total number of targets
     */
    public int getTotalTargetCount() {
        int total = 0;
        for (PhotonVisionIOInputsAutoLogged input : inputs) {
            total += input.targetCount;
        }
        return total;
    }

    /**
     * Projects a 2D detection to 3D pose relative to the robot center
     * Uses raw detection data including pixel coordinates, pitch, yaw, skew, and area
     * @param detection The raw detection data
     * @param cameraResolutionX Camera resolution width in pixels
     * @param cameraResolutionY Camera resolution height in pixels
     * @return Estimated 3D pose of the target relative to the robot center
     */
    public Pose3d projectTargetTo3D(RawDetection detection, int cameraResolutionX, int cameraResolutionY) {
        // Get camera constants from RobotConstants
        double cameraHeightMeters = RobotConstants.PhotonvisionConstants.CAMERA_HEIGHT_METERS.get();
        double cameraPitchDegrees = RobotConstants.PhotonvisionConstants.CAMERA_PITCH_DEGREES.get();
        
        // Convert angles to radians
        double yawRadians = Units.degreesToRadians(detection.yaw);
        double pitchRadians = Units.degreesToRadians(detection.pitch);
        double cameraPitchRadians = Units.degreesToRadians(cameraPitchDegrees);
        double skewRadians = Units.degreesToRadians(detection.skew);
        
        // Get tunable parameters
        double groundHeight = RobotConstants.PhotonvisionConstants.GROUND_HEIGHT_METERS.get();
        
        // Calculate distance using trigonometry only (reliable for irregular shapes)
        double distance = 1.0; // Default fallback
        double heightDifference = cameraHeightMeters - groundHeight;
        double totalPitch = pitchRadians + cameraPitchRadians;
        
        if (Math.abs(totalPitch) > 0.01) { // Avoid division by zero
            distance = Math.abs(heightDifference / Math.tan(totalPitch));
            
            // Apply distance scale factor for fine-tuning
            double distanceScaleFactor = RobotConstants.PhotonvisionConstants.DISTANCE_SCALE_FACTOR.get();
            distance *= distanceScaleFactor;
        }
        
        // Calculate 3D position - objects are always on the ground
        // Use horizontal distance projection for x and y
        double x = distance * Math.cos(yawRadians);  // Forward/backward distance
        double y = -distance * Math.sin(yawRadians); // Left/right distance (negated because PhotonVision +yaw = right, robot +Y = left)
        // Z represents height relative to camera - since objects are on ground, use negative camera height
        double z = -(cameraHeightMeters - groundHeight);
        
        // Create camera-relative pose
        Translation3d cameraRelativeTranslation = new Translation3d(x, y, z);
        Rotation3d cameraRelativeRotation = new Rotation3d(0, 0, 0);
        Pose3d cameraRelativePose = new Pose3d(cameraRelativeTranslation, cameraRelativeRotation);
        
        // Transform from camera coordinates to robot coordinates
        Pose3d robotRelativePose = transformCameraToRobot(cameraRelativePose);
        
        // Log the projection calculation for debugging
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/YawDegrees", detection.yaw);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/Distance", distance);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/HeightDifference", heightDifference);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/CameraRelativeX", x);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/CameraRelativeY", y);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/CameraRelativeZ", z);
        
        // Log coordinate system verification for debugging
        String yawDirection = detection.yaw > 0 ? "RIGHT" : (detection.yaw < 0 ? "LEFT" : "CENTER");
        String yDirection = y > 0 ? "LEFT" : (y < 0 ? "RIGHT" : "CENTER");
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Debug/YawDirection", yawDirection);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Debug/YCoordinateDirection", yDirection);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/RobotRelativeX", robotRelativePose.getX());
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/RobotRelativeY", robotRelativePose.getY());
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Projection/RobotRelativeZ", robotRelativePose.getZ());
        
        return robotRelativePose;
    }
    
    /**
     * Transforms a pose from camera coordinates to robot coordinates
     * @param cameraRelativePose The pose relative to the camera
     * @return The pose relative to the robot center
     */
    private Pose3d transformCameraToRobot(Pose3d cameraRelativePose) {
        // Get camera position relative to robot center from constants
        double cameraToRobotX = RobotConstants.PhotonvisionConstants.CAMERA_TO_ROBOT_X.get();
        double cameraToRobotY = RobotConstants.PhotonvisionConstants.CAMERA_TO_ROBOT_Y.get();
        double cameraToRobotZ = RobotConstants.PhotonvisionConstants.CAMERA_TO_ROBOT_Z.get();
        double cameraToRobotRotationDegrees = RobotConstants.PhotonvisionConstants.CAMERA_TO_ROBOT_ROTATION_DEGREES.get();
        
        // Create the camera-to-robot transform
        Translation3d cameraToRobotTranslation = new Translation3d(cameraToRobotX, cameraToRobotY, cameraToRobotZ);
        Rotation3d cameraToRobotRotation = new Rotation3d(0, 0, Units.degreesToRadians(cameraToRobotRotationDegrees));
        Transform3d cameraToRobotTransform = new Transform3d(cameraToRobotTranslation, cameraToRobotRotation);
        
        // Apply the transform to get robot-relative pose
        return cameraRelativePose.transformBy(cameraToRobotTransform);
    }
    
    /**
     * Projects all detected targets to 3D poses relative to the robot center
     * @param cameraResolutionX Camera resolution width in pixels
     * @param cameraResolutionY Camera resolution height in pixels
     * @return List of 3D poses for all detected targets relative to robot center
     */
    public List<Pose3d> projectAllTargetsTo3D(int cameraResolutionX, int cameraResolutionY) {
        List<Pose3d> poses = new ArrayList<>();
        
        for (RawDetection detection : getAllRawDetections()) {
            poses.add(projectTargetTo3D(detection, cameraResolutionX, cameraResolutionY));
        }
        
        return poses;
    }



    /**
     * Record class to hold raw detection data
     */
    public record RawDetection(
        int cameraId,
        long timestampMs,
        double yaw,         // degrees, positive = right
        double pitch,       // degrees, positive = up
        double area,        // percent (0-100)
        double skew,        // degrees
        double ambiguity,   // 0-1, lower is better
        int fiducialId,     // -1 for objects, positive for AprilTags
        double pixelX,      // pixel X coordinate of target center
        double pixelY       // pixel Y coordinate of target center
    ) {
        
        public boolean isAprilTag() {
            return fiducialId >= 0;
        }
        
        public boolean isObject() {
            return fiducialId < 0;
        }
        
        public double getConfidence() {
            return Math.max(0.0, 1.0 - ambiguity);
        }
    }
}
