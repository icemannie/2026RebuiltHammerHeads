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

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean frontConnected = false;
        public Angle frontPosition = Radians.of(0);
        public AngularVelocity frontVelocity = RadiansPerSecond.of(0);
        public Current frontTorqueCurrent = Amps.of(0);
        public Current frontSupplyCurrent = Amps.of(0);
        public Voltage frontAppliedVoltage = Volts.of(0);

        public boolean backConnected = false;
        public Angle backPosition = Radians.of(0);
        public AngularVelocity backVelocity = RadiansPerSecond.of(0);
        public Current backTorqueCurrent = Amps.of(0);
        public Current backSupplyCurrent = Amps.of(0);
        public Voltage backAppliedVoltage = Volts.of(0);

        public Angle averagePosition = Radians.of(0);
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setVoltage(Voltage out) {}

    public default void setFrontVoltage(Voltage out) {}

    public default void setBackVoltage(Voltage out) {}

    public default void stop() {}

    public default void stopFront() {}

    public default void stopBack() {}

    public default void zeroPosition() {}

    public default void setNeutralMode(NeutralModeValue neutralModeValue) {}
}
