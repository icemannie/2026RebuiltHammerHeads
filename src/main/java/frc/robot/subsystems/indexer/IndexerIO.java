package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean spinMotorConnected = false;
        public AngularVelocity spinVelocity = RadiansPerSecond.zero();
        public Current spinCurrent = Amps.zero();
        public Voltage spinAppliedVolts = Volts.zero();

        public boolean feedMotorConnected = false;
        public AngularVelocity feedVelocity = RadiansPerSecond.zero();
        public Current feedCurrent = Amps.zero();
        public Voltage feedAppliedVolts = Volts.zero();
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setSpinOutput(Voltage out) {}

    public default void setFeedOutput(Voltage out) {}

    public default void stopSpin() {}

    public default void stopFeed() {}
}
