package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HookIO {
    @AutoLog
    public static class HookIOInputs {
        public boolean connected = false;
        public Angle position = Radians.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        public Current torqueCurrent = Amps.of(0);
        public Voltage appliedVoltage = Volts.of(0);
    }

    public default void updateInputs(HookIOInputs inputs) {}

    public default void setVoltage(Voltage out) {}

    public default void zeroPosition() {}

    public default void setNeutralMode(NeutralModeValue neutralModeValue) {}
}
