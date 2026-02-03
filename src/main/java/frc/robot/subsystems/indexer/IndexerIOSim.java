package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOSim implements IndexerIO {
    AngularVelocity simulatedSpinVelocity = RotationsPerSecond.zero();

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spinVelocity = simulatedSpinVelocity;
    }

    @Override
    public void setSpinOutput(Voltage out) {
        simulatedSpinVelocity = RotationsPerSecond.of(0.5 * out.in(Volts));
    }

    @Override
    public void stopSpin() {
        simulatedSpinVelocity = RotationsPerSecond.zero();
    }
}
