package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IndexerVisualizer {
    private Angle spinAngle = Radians.zero();
    private final Supplier<Rotation2d> robotRotationSupplier;

    public IndexerVisualizer(Supplier<Rotation2d> robotRotationSupplier) {
        this.robotRotationSupplier = robotRotationSupplier;
    }

    public void update(AngularVelocity spinVelocity) {
        spinAngle = spinAngle.plus(Seconds.of(0.02).times(spinVelocity));

        Logger.recordOutput(
                "Indexer/Pose",
                new Pose2d(new Translation2d(), new Rotation2d(spinAngle).plus(robotRotationSupplier.get())));
    }
}
