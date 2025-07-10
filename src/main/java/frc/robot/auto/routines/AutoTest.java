package frc.robot.auto.routines;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import lib.ironpulse.utils.Logging;

public class AutoTest extends AutoRoutine {
  private static PathPlannerPath testPath;

  public AutoTest() {
    super("Test Auto");
    try {
      testPath = PathPlannerPath.fromPathFile("Test Path");
    } catch (Exception e) {
      Logging.error("Auto/TestAuto", "Failed to load auto! %s", e.getMessage());
    }

  }

  @Override
  public Command getAutoCommand() {
    return AutoActions.followPath(testPath);
  }

  @Override
  public Command getOnSelectCommand() {
    return AutoActions.resetOnPathStart(testPath);
  }
}
