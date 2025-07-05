package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Currently it is designed for object detection only.
 */
public interface PhotonVisionIO {

    default void updateInputs(PhotonVisionIOInputs inputs) {
    }

    void takeOutputSnapshot();

    @AutoLog
    class PhotonVisionIOInputs {
        public String name;
        public int id;
        public boolean connected = false;
        public int lastObservedPeriod = 0;
        public Pose2d nearestCoralPosition = null;
        public double lastObservedConf = 0.0;
    }

}