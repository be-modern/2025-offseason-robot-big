package frc.robot.subsystems.photonvision;

import static frc.robot.RobotConstants.PhotonvisionConstants.PV_CAMERA_NAMES;

public class PhotonVisionIOSim implements PhotonVisionIO {

    private boolean connected = true;
    private String name;
    private int id;

    public PhotonVisionIOSim(int id) {
        this.id = id;
        this.name = PV_CAMERA_NAMES[id];
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        inputs.connected = connected;
        inputs.name = name;
        inputs.nearestCoralPosition = null;//TODO: random coral position based on alliance color
    }

    @Override
    public void takeOutputSnapshot() {
        return;
    }
}
