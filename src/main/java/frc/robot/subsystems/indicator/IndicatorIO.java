package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.AddressableLEDPattern;
import frc.robot.drivers.led.patterns.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Seconds;

public interface IndicatorIO {
    default Color allianceColor() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Blue -> Color.kBlue;
            case Red -> Color.kRed;
            default -> Color.kWhite;
        };
    }

    default void updateInputs(IndicatorIOInputs inputs) {}

    default void setPattern(Patterns pattern) {}

    default void reset() {}

    enum Patterns {
        NORMAL(new BlinkingPattern(Color.kBlue, 0.5)),
        INTAKE(new BlinkingPattern(Color.kRed, 0.02)),
        AFTER_INTAKE(new BlinkingPattern(Color.kGreen, 0.02)),
        RESET_ODOM(new BlinkingPattern(Color.kWhite, 0.1)),
        AIMING(new BlinkingPattern(Color.kBlue, 0.02)),
        AIMED(new SolidColorPattern(Color.kBlue));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern = Patterns.NORMAL;
    }
}
