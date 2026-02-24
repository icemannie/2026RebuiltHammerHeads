package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

public class IndexerVisualizer {
    private Angle spinAngle = Radians.zero();

    public IndexerVisualizer() {}

    public void update(AngularVelocity spinVelocity) {
        spinAngle = spinAngle.plus(Seconds.of(0.02).times(spinVelocity.unaryMinus()));

        Logger.recordOutput("Indexer/Pose", new Pose3d(new Pose2d(new Translation2d(), new Rotation2d(spinAngle))));
    }
}
