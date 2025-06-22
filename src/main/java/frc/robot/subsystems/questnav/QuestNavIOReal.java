package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotConstants;
import frc.robot.utils.TunableNumber;

// Import the QuestNav class from vendor dependency
// import gg.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {
    // private final QuestNav questNav;
    
    // Transform from robot center to Quest headset
    private final TunableNumber robotToQuestX = new TunableNumber("QuestNav/RobotToQuestX", 0.0);
    private final TunableNumber robotToQuestY = new TunableNumber("QuestNav/RobotToQuestY", 0.0);
    private final TunableNumber robotToQuestRotDeg = new TunableNumber("QuestNav/RobotToQuestRotDeg", 0.0);

    public QuestNavIOReal() {
        // questNav = new QuestNav();
        System.out.println("QuestNavIOReal initialized - QuestNav vendor library not yet imported");
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        // TODO: Uncomment when QuestNav vendor library is available
        // inputs.connected = questNav.isConnected();
        // inputs.tracking = questNav.isTracking();
        // inputs.pose = questNav.getPose();
        // inputs.timestamp = questNav.getDataTimestamp();
        
        // Temporary default values
        inputs.connected = false;
        inputs.tracking = false;
        inputs.pose = new Pose2d();
        inputs.timestamp = 0.0;
        
        System.out.println("QuestNavIOReal: Waiting for QuestNav vendor library to be imported");
    }

    @Override
    public void setPose(Pose2d robotPose) {
        // Transform robot pose to Quest pose
        Transform2d robotToQuest = new Transform2d(
            robotToQuestX.get(),
            robotToQuestY.get(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(robotToQuestRotDeg.get())
        );
        
        Pose2d questPose = robotPose.transformBy(robotToQuest);
        
        // TODO: Uncomment when QuestNav vendor library is available
        // questNav.setPose(questPose);
        
        System.out.println("QuestNavIOReal: setPose called with robot pose: " + robotPose + ", quest pose: " + questPose);
    }

    @Override
    public void commandPeriodic() {
        // TODO: Uncomment when QuestNav vendor library is available
        // questNav.commandPeriodic();
        
        // This is critical for QuestNav v2025-1.0.0+ to function properly
        System.out.println("QuestNavIOReal: commandPeriodic() called - waiting for vendor library");
    }
} 