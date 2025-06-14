package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.utils.LoggedTracer;
import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_SPOOL_DIAMETER;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem  {
    @Getter
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(ElevatorConstants.ELEVATOR_ZEROING_FILTER_SIZE);
    @AutoLogOutput(key = "Elevator/currentFilterValue")
    public double currentFilterValue = 0.0;
    @Getter
    @AutoLogOutput(key = "Elevator/zeroing")
    public boolean zeroing = false;
    @Getter
    @AutoLogOutput(key = "Elevator/setPoint")
    private double wantedPosition = 0.16;
    @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;
    @AutoLogOutput(key = "Elevator/stopDueToLimit")
    private boolean stopDueToLimit = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        
        // Check if position exceeds maximum extension
        if (wantedPosition > ElevatorConstants.MAX_EXTENSION_METERS.get()) {
            stopDueToLimit = true;
            throw new IllegalArgumentException("Elevator setpoint " + wantedPosition + " exceeds maximum extension of " + 
                ElevatorConstants.MAX_EXTENSION_METERS.get() + " meters");
        } else if (stopDueToLimit) {
            // Reset stopDueToLimit if position is now valid
            stopDueToLimit = false;
        }

        final boolean runningGoal = !stopDueToLimit && !zeroing;
        if (runningGoal) {
            atGoal = elevatorAtGoal(ElevatorConstants.ELEVATOR_GOAL_TOLERANCE.get());
            io.setElevatorTarget(wantedPosition);
        } else {
            atGoal = false;
        }
        LoggedTracer.record("Elevator");
    }

    public double getElevatorPosition() {
        return inputs.positionMeters;
    }

    public void setElevatorPosition(DoubleSupplier position) {
        wantedPosition = position.getAsDouble();
    }

    public boolean elevatorAtGoal(double offset) {
        return Math.abs(inputs.positionMeters - wantedPosition) < offset;
    }

    public Command zeroElevator() {
        return Commands.startRun(
            () -> {
                zeroing = true;
            },
            () -> {
                if (RobotBase.isReal()) {
                    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
                    if (currentFilterValue <= ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
                        io.setElevatorVoltage(-1);
                    }
                    if (currentFilterValue > ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
                        io.setElevatorVoltage(0);
                        io.resetElevatorPosition();
                        zeroing = false;
                    }
                } else {
                    io.setElevatorTarget(0);
                    if (Math.abs(inputs.positionMeters) < 0.01) {
                        zeroing = false;
                    }
                }
            })
            .until(() -> !zeroing)
            .finallyDo(() -> {
                zeroing = false;
            });
    }

    public boolean isSafeToFlip() {
        return (inputs.positionMeters > ElevatorConstants.SAFE_HEIGHT_FLIP.get());
    }
}