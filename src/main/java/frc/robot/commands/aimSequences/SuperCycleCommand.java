package frc.robot.commands.aimSequences;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;

public class SuperCycleCommand extends SequentialCommandGroup {
    private final Superstructure superstructure;
    private final IndicatorSubsystem indicatorSubsystem;
    private final CommandXboxController driverController;
    private BooleanSupplier stop;

    public SuperCycleCommand(Superstructure superstructure,
                             IndicatorSubsystem indicatorSubsystem,
                             CommandXboxController driverController,
                             BooleanSupplier stop) {
        this.indicatorSubsystem = indicatorSubsystem;
        this.superstructure = superstructure;
        this.driverController = driverController;
        this.stop = stop;

        addRequirements(superstructure);

        addCommands(
                Commands.either(
                        // preshoot and shoot coral, then take the algae
                        Commands.sequence(
                                // preshoot coral(press right trigger to end)
                                preShootCoral(),
                                // shoot
                                superstructure
                                        .runGoal(
                                                () -> DestinationSupplier
                                                        .getInstance()
                                                        .ejectStateTransition()
                                        ).until(() -> !superstructure.hasCoral()),
                                // take algae
                                takeAlgae()
                        ),
                        // if no coral, just take algae
                        takeAlgae(),
                        // check if the superstructure has coral
                        superstructure::hasCoral
                )
        );
    }


// whenever the right trigger is pressed, the preShootCoral Command will end and
// continue to shoot
    private Command preShootCoral() {
        return Commands.parallel(
                // move the robot to the correct position
                Commands.sequence(
                        new ReefAimCommand(stop, superstructure, driverController, indicatorSubsystem),
                        Commands.waitSeconds(0.3)
                ),

                // set up the elevator and end effector
                superstructure
                        .runGoal(() -> DestinationSupplier
                                .getInstance()
                                .getCurrentElevSetpointCoral())
                        .until(() ->
                                (driverController.rightTrigger().getAsBoolean() && Robot.isReal()))
        );
    }

    private Command takeAlgae() {
        DestinationSupplier.getInstance().updatePokeSetpointByTag(
                DestinationSupplier.getNearestTagID(
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp())));
        var safeToRaise = DestinationSupplier.isSafeToRaise(
                Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()),
                DestinationSupplier.getInstance().getCurrentBranch());


        return Commands.parallel(
                // change the current game piece
                Commands.runOnce(() ->
                        DestinationSupplier.getInstance().setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING)),
                // move to the correct position
                new ReefAimCommand(stop, superstructure, driverController, indicatorSubsystem),
                // set up ee and elevator and take the algae
                superstructure
                        .runGoal(() -> DestinationSupplier
                                .getInstance()
                                .getCurrentElevSetpointAlgae())
                        .until(superstructure::hasAlgae)
        ).onlyIf(() -> safeToRaise)
                .onlyIf(() -> DestinationSupplier.getInstance().useSuperCycle);
    }
}
