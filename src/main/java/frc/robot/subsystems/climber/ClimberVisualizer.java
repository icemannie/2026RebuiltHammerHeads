package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberVisualizer {
    private final double angleRad = Units.degreesToRadians(55);
    private final double maxPos = Units.inchesToMeters(12);
    private final double meterPerRad = maxPos / ClimberConstants.EXTEND_POSITION_FRONT.in(Radians);

    public ClimberVisualizer() {}

    public void update(Angle frontPos, Angle backPos) {
        Logger.recordOutput(
                "Climber/Front Pose",
                new Pose3d(
                        new Translation3d(
                                0,
                                -meterPerRad * frontPos.in(Radians) * Math.cos(angleRad),
                                meterPerRad * frontPos.in(Radians) * Math.sin(angleRad)),
                        Rotation3d.kZero));
        Logger.recordOutput(
                "Climber/Back Pose",
                new Pose3d(
                        new Translation3d(
                                0,
                                -meterPerRad * backPos.in(Radians) * Math.cos(angleRad),
                                meterPerRad * backPos.in(Radians) * Math.sin(angleRad)),
                        Rotation3d.kZero));
    }
}
