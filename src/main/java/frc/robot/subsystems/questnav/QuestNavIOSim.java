package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class QuestNavIOSim implements QuestNavIO {
    private Pose2d simulatedPose = new Pose2d();
    private boolean simulatedConnected = true;
    private boolean simulatedTracking = true;

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.connected = simulatedConnected;
        inputs.tracking = simulatedTracking;
        inputs.pose = simulatedPose;
        inputs.timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedPose = pose;
        System.out.println("QuestNavIOSim: setPose called with: " + pose);
    }

    @Override
    public void commandPeriodic() {
        // Simulation doesn't need to do anything special for commandPeriodic
    }
} 