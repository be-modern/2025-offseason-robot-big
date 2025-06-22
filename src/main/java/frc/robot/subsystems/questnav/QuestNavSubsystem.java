package frc.robot.subsystems.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();
    
    // Standard deviations for pose estimation
    private final TunableNumber stdDevX = new TunableNumber("QuestNav/StdDevX", 0.02);
    private final TunableNumber stdDevY = new TunableNumber("QuestNav/StdDevY", 0.02);
    private final TunableNumber stdDevRot = new TunableNumber("QuestNav/StdDevRotDeg", 2.0);
    
    // Transform from robot center to Quest headset
    private final TunableNumber robotToQuestX = new TunableNumber("QuestNav/RobotToQuestX", 0.0);
    private final TunableNumber robotToQuestY = new TunableNumber("QuestNav/RobotToQuestY", 0.0);
    private final TunableNumber robotToQuestRotDeg = new TunableNumber("QuestNav/RobotToQuestRotDeg", 0.0);
    
    // Enable/disable vision updates
    private final TunableNumber enableVisionUpdates = new TunableNumber("QuestNav/EnableVisionUpdates", 1.0);
    
    private final Swerve swerve;

    public QuestNavSubsystem(QuestNavIO io) {
        this.io = io;
        this.swerve = Swerve.getInstance();
        
        Logger.recordOutput("QuestNav/Status", "Initialized");
    }

    @Override
    public void periodic() {
        // Update inputs and log them
        io.updateInputs(inputs);
        Logger.processInputs("QuestNav", inputs);
        
        // CRITICAL: Call commandPeriodic() for QuestNav v2025-1.0.0+
        io.commandPeriodic();
        
        // Add vision measurement to swerve drive if conditions are met
        addVisionMeasurement();
        
        // Log important outputs
        Logger.recordOutput("QuestNav/RobotPose", getRobotPose());
        Logger.recordOutput("QuestNav/QuestPose", inputs.pose);
        Logger.recordOutput("QuestNav/Connected", inputs.connected);
        Logger.recordOutput("QuestNav/Tracking", inputs.tracking);
    }

    /**
     * Gets the robot pose by transforming the Quest pose
     * @return Robot pose on the field
     */
    @AutoLogOutput(key = "QuestNav/RobotPose")
    public Pose2d getRobotPose() {
        Transform2d robotToQuest = getRobotToQuestTransform();
        return inputs.pose.transformBy(robotToQuest.inverse());
    }

    /**
     * Sets the robot pose by transforming it to Quest coordinates
     * @param robotPose The robot pose to set
     */
    public void resetPose(Pose2d robotPose) {
        resetPose(robotPose, false);
    }

    /**
     * Sets the robot pose by transforming it to Quest coordinates
     * @param robotPose The robot pose to set
     * @param overrideEnabledCheck Whether to allow resetting while enabled
     */
    public void resetPose(Pose2d robotPose, boolean overrideEnabledCheck) {
        if (!overrideEnabledCheck && DriverStation.isEnabled()) {
            Logger.recordOutput("QuestNav/Warning", 
                "resetPose() called while robot is enabled. Ignoring for safety.");
            System.out.println("QuestNav: Cannot reset pose while robot is enabled!");
            return;
        }

        Transform2d robotToQuest = getRobotToQuestTransform();
        Pose2d questPose = robotPose.transformBy(robotToQuest);
        
        io.setPose(questPose);
        
        Logger.recordOutput("QuestNav/Status", 
            String.format("Reset pose - Robot: %s, Quest: %s", robotPose, questPose));
        System.out.println("QuestNav: Reset pose to " + robotPose);
    }

    /**
     * Checks if the Quest is connected
     * @return true if connected
     */
    public boolean isConnected() {
        return inputs.connected;
    }

    /**
     * Checks if the Quest is tracking
     * @return true if tracking
     */
    public boolean isTracking() {
        return inputs.tracking;
    }

    /**
     * Gets the timestamp of the latest pose data
     * @return timestamp in seconds
     */
    public double getTimestamp() {
        return inputs.timestamp;
    }

    /**
     * Adds vision measurement to swerve drive pose estimator if conditions are met
     */
    private void addVisionMeasurement() {
        // Only add vision measurement if enabled and Quest is connected and tracking
        if (enableVisionUpdates.get() == 0.0 || !inputs.connected || !inputs.tracking) {
            return;
        }

        // Get robot pose from Quest data
        Pose2d robotPose = getRobotPose();
        
        // Create standard deviation matrix
        Matrix<N3, N1> stdDevs = VecBuilder.fill(
            stdDevX.get(),
            stdDevY.get(),
            Math.toRadians(stdDevRot.get())
        );

        // Add measurement to swerve drive pose estimator through localizer
        swerve.getLocalizer().addMeasurement(inputs.timestamp, robotPose, stdDevs);
        
        Logger.recordOutput("QuestNav/VisionMeasurementAdded", true);
    }

    /**
     * Gets the transform from robot center to Quest headset
     * @return Transform2d representing the offset
     */
    private Transform2d getRobotToQuestTransform() {
        return new Transform2d(
            robotToQuestX.get(),
            robotToQuestY.get(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(robotToQuestRotDeg.get())
        );
    }
} 