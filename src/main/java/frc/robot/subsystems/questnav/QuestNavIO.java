package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
    @AutoLog
    public static class QuestNavIOInputs {
        public boolean connected = false;
        public boolean tracking = false;
        public Pose2d pose = new Pose2d();
        public double timestamp = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(QuestNavIOInputs inputs) {}

    /** Sets the robot pose */
    public default void setPose(Pose2d pose) {}

    /** Calls the periodic update for QuestNav */
    public default void commandPeriodic() {}

    /** Default implementation for simulation/replay */
    public class Default implements QuestNavIO {}
} 