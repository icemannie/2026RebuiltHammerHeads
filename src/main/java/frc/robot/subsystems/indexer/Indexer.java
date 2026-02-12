package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.FEED_THRESHOLD;
import static frc.robot.Constants.IndexerConstants.FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.SPIN_VOLTAGE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.Set;
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
    private IndexerGoal goal = IndexerGoal.OFF;

    public Indexer(IndexerIO io, Supplier<Rotation2d> robotRotationSupplier) {
        this.io = io;
        this.visualizer = new IndexerVisualizer(robotRotationSupplier);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        if ((spinVoltage.hasChanged(hashCode()) || feedVoltage.hasChanged(hashCode())) && goal == IndexerGoal.ACTIVE) {
            io.setSpinOutput(Volts.of(spinVoltage.get()));
            io.setFeedOutput(Volts.of(feedVoltage.get()));
        }

        visualizer.update(inputs.spinVelocity);
    }

    public Command setGoal(IndexerGoal goal) {
        return Commands.defer(
                () -> {
                    Command toSchedule = Commands.none();
                    if (goal == IndexerGoal.ACTIVE && this.goal != IndexerGoal.ACTIVE) {
                        toSchedule = activate();
                    } else if (goal == IndexerGoal.OFF) {
                        toSchedule = stop();
                    }
                    this.goal = goal;
                    return toSchedule;
                },
                Set.of(this));
    }

    public Command activate() {
        return this.runOnce(() -> {
                    io.setFeedOutput(Volts.of(-4));
                    io.setSpinOutput(Volts.of(-2));
                })
                .andThen(Commands.waitSeconds(0.1875))
                .andThen(this.runOnce(() -> {
                    io.setFeedOutput(Volts.of(feedVoltage.get()));
                    io.stopSpin();
                }))
                .andThen(Commands.waitUntil(() -> inputs.feedVelocity.abs(RPM) >= FEED_THRESHOLD.in(RPM)))
                .andThen(this.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltage.get()))))
                .withName("IndexerActivate");
    }

    public Command stop() {
        return this.runOnce(() -> {
                    io.stopSpin();
                    io.stopFeed();
                })
                .withName("IndexerStop");
    }

    public enum IndexerGoal {
        ACTIVE,
        OFF
    }
}
