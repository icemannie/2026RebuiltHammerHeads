package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.SPIN_VOLTAGE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final LoggedTunableNumber spinVoltage =
            new LoggedTunableNumber("Indexer/Spin Voltage", SPIN_VOLTAGE.in(Volts));
    private final LoggedTunableNumber feedVoltage =
            new LoggedTunableNumber("Indexer/Feed Voltage", FEED_VOLTAGE.in(Volts));

    private final IndexerVisualizer visualizer;

    @AutoLogOutput
    private boolean isActive = false;

    public Indexer(IndexerIO io, Supplier<Rotation2d> robotRotationSupplier) {
        this.io = io;
        this.visualizer = new IndexerVisualizer(robotRotationSupplier);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        if (isActive && (spinVoltage.hasChanged(hashCode()) || feedVoltage.hasChanged(hashCode()))) {
            io.setSpinOutput(Volts.of(spinVoltage.get()));
            io.setFeedOutput(Volts.of(feedVoltage.get()));
        }

        visualizer.update(inputs.spinVelocity);
    }

    public Command activate() {
        return this.runOnce(() -> {
                    io.setSpinOutput(Volts.of(spinVoltage.get()));
                    io.setFeedOutput(Volts.of(feedVoltage.get()));
                    isActive = true;
                })
                .withName("IndexerActivate");
    }

    public Command stop() {
        return this.runOnce(() -> {
                    io.stopSpin();
                    io.stopFeed();
                    isActive = false;
                })
                .withName("IndexerStop");
    }
}
