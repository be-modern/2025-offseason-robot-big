# QuestNav Subsystem - New Implementation

This folder contains the new QuestNav subsystem implementation following the QuestNav v2025-1.0.0+ API.

## What Was Done

### 1. Hardware Abstraction Structure
- **QuestNavIO.java**: Interface defining the hardware abstraction layer
- **QuestNavIOReal.java**: Real hardware implementation (placeholder for actual QuestNav library)
- **QuestNavIOSim.java**: Simulation implementation
- **QuestNavSubsystem.java**: Main subsystem class with periodic updates and vision measurement integration

### 2. Integration with Robot Code
- Updated `RobotContainer.java` to use the new QuestNav classes
- Added QuestNav constants to `RobotConstants.java`
- Integrated with swerve drive pose estimator for vision corrections

### 3. Features Implemented
- ✅ Hardware abstraction layer (IO pattern)
- ✅ Pose estimation with configurable standard deviations
- ✅ Transform handling between robot center and Quest headset
- ✅ Integration with swerve drive localizer
- ✅ Safety checks (no pose reset while enabled)
- ✅ Comprehensive logging with AdvantageKit
- ✅ Tunable parameters for easy adjustment

## What Still Needs To Be Done

### 1. Install QuestNav Vendor Library
The actual QuestNav vendor library needs to be installed and imported:

1. Download the latest `questnavlib.json` from [QuestNav GitHub releases](https://github.com/questnav/questnav/releases)
2. Add it to the `vendordeps` folder
3. Rebuild the project
4. Uncomment the QuestNav import in `QuestNavIOReal.java`:
   ```java
   // import gg.questnav.QuestNav;
   ```
5. Uncomment all the QuestNav API calls in `QuestNavIOReal.java`

### 2. Measure and Configure Transform
Update the robot-to-Quest transform constants in `RobotConstants.QuestNavConstants`:
- `ROBOT_TO_QUEST_X`: X offset from robot center to Quest headset (meters)
- `ROBOT_TO_QUEST_Y`: Y offset from robot center to Quest headset (meters) 
- `ROBOT_TO_QUEST_ROT_DEG`: Rotational offset (degrees)

### 3. Tune Standard Deviations
Adjust the pose estimation standard deviations based on testing:
- `STD_DEV_X`: Trust level in X direction (default: 0.02m)
- `STD_DEV_Y`: Trust level in Y direction (default: 0.02m)
- `STD_DEV_ROT_DEG`: Trust level in rotation (default: 2.0 degrees)

### 4. Test and Validate
- Test pose reset functionality with Stream Deck button 1
- Verify pose measurements are being added to swerve localizer
- Monitor AdvantageKit logs for Quest connection and tracking status
- Validate transform calculations are correct

## Usage

### Initialization
The QuestNav subsystem is automatically initialized in `RobotContainer.java`:
- Real robot: Uses `QuestNavIOReal`
- Simulation: Uses `QuestNavIOSim` 
- Replay: Uses default implementation

### Pose Reset
Button 1 on the Stream Deck resets the Quest pose to the current swerve localizer pose:
```java
streamDeckController.button(1).onTrue(
    Commands.runOnce(() -> questNavSubsystem.resetPose(
        swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()), 
        true
    ))
);
```

### Monitoring
Check AdvantageKit logs for:
- `QuestNav/Connected`: Quest connection status
- `QuestNav/Tracking`: Quest tracking status  
- `QuestNav/RobotPose`: Calculated robot pose from Quest
- `QuestNav/VisionMeasurementAdded`: Whether measurements are being added

## Key Differences from Old Implementation

1. **New API**: Uses the official QuestNav vendor library instead of custom NetworkTables
2. **Simplified**: Removes complex retry mechanisms and custom protocols
3. **Better Integration**: Direct integration with swerve localizer like LimelightSubsystem
4. **Modern Pattern**: Follows current team hardware abstraction patterns
5. **Required Periodic Call**: Must call `commandPeriodic()` for v2025-1.0.0+ to function

## References
- [QuestNav Documentation](https://questnav.gg/docs/getting-started/robot-code/)
- [QuestNav GitHub](https://github.com/questnav/questnav)
- Team 6941 Hardware Abstraction Patterns 