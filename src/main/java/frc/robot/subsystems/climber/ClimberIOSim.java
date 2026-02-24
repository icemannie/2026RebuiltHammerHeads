package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {
    private final double maxPosRad = Math.max(
            ClimberConstants.EXTEND_POSITION_FRONT.in(Radians), ClimberConstants.EXTEND_POSITION_BACK.in(Radians));
    private final double maxSpeedTravelTime = 0.5;
    private final double maxSpeedRadPerSec = maxPosRad / maxSpeedTravelTime;
    private final double kV = maxSpeedRadPerSec / 12; // rad/s/V
    private double position = 0;
    private double velocity = 0;
    private double voltage = 0;

    public ClimberIOSim() {}

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        position += velocity * 0.02;
        if (position > maxPosRad || position < 0) {
            velocity = 0;
            position = MathUtil.clamp(position, 0, maxPosRad);
            inputs.frontTorqueCurrent = Amps.of(60);
            inputs.backTorqueCurrent = Amps.of(60);
        } else {
            inputs.frontTorqueCurrent = Amps.of(0);
            inputs.backTorqueCurrent = Amps.of(0);
        }

        inputs.frontPosition = Radians.of(position);
        inputs.frontVelocity = RadiansPerSecond.of(velocity);
        inputs.frontAppliedVoltage = Volts.of(voltage);

        inputs.backPosition = Radians.of(position);
        inputs.backVelocity = RadiansPerSecond.of(velocity);
        inputs.backAppliedVoltage = Volts.of(voltage);

        inputs.averagePosition = Radians.of(position);
    }

    @Override
    public void setVoltage(Voltage out) {
        voltage = out.in(Volts);
        velocity = voltage * kV;
    }

    @Override
    public void setFrontVoltage(Voltage out) {
        setVoltage(out);
    }

    @Override
    public void setBackVoltage(Voltage out) {
        setVoltage(out);
    }

    @Override
    public void stop() {
        setVoltage(Volts.of(0));
    }

    // @Override
    // public void stopFront() {
    //     stop();
    // }

    // @Override
    // public void stopBack() {
    //     stop();
    // }
}
