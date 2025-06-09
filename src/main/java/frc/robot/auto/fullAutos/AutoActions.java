package frc.robot.auto.fullAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.basics.FollowPath;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;

public class AutoActions {
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final Swerve swerve;
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();

    public AutoActions(IndicatorSubsystem indicatorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = Swerve.getInstance();
    }

    // invoke event marker
    public void invokeCommand(String name, BooleanSupplier stopSupplier) {
        switch (name) {

        }
    }

    public Command followPath(PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        return new FollowPath(this, swerve, path, angleLock, requiredOnTarget, resetOdometry);
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public Command setL4() {
        return Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(SuperstructureState.L4));
    }

    public Command setLevel(SuperstructureState setpoint) {
        return Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(setpoint));
    }
}
