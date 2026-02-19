package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.CLIMB_POSITION;
import static frc.robot.Constants.ClimberConstants.CLIMB_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.EXTEND_POSITION;
import static frc.robot.Constants.ClimberConstants.EXTEND_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.ClimberConstants.STALL_CURRENT;
import static frc.robot.Constants.ClimberConstants.STOW_POSITION;
import static frc.robot.Constants.ClimberConstants.STOW_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.ZERO_VOLTAGE;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hook extends SubsystemBase {
    private final HookIO io;
    private final HookIOInputsAutoLogged inputs = new HookIOInputsAutoLogged();
    private final String name;

    public Hook(HookIO io, String name) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/" + name, inputs);
    }

    private Command setVoltage(Voltage out) {
        return this.runOnce(() -> io.setVoltage(out));
    }

    public Command climb() {
        return Commands.sequence(
                setVoltage(CLIMB_VOLTAGE),
                Commands.waitUntil(() -> inputs.position.lte(CLIMB_POSITION)),
                setVoltage(Volts.zero()));
    }

    public Command stow() {
        return Commands.sequence(
                setVoltage(STOW_VOLTAGE),
                Commands.waitUntil(() -> inputs.position.lte(STOW_POSITION)),
                setVoltage(Volts.zero()));
    }

    public Command extend() {
        return Commands.sequence(
                setVoltage(EXTEND_VOLTAGE),
                Commands.waitUntil(() -> inputs.position.gte(EXTEND_POSITION)),
                setVoltage(Volts.zero()));
    }

    public Command zero() {
        return Commands.sequence(
                setVoltage(ZERO_VOLTAGE),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(() -> inputs.torqueCurrent.abs(Amps) > STALL_CURRENT.abs(Amps)
                        && inputs.velocity.abs(RadiansPerSecond) < STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond)),
                setVoltage(Volts.zero()),
                Commands.waitSeconds(0.1),
                this.runOnce(io::zeroPosition));
    }
}
