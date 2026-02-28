package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.AUTO_CLIMB_POSITION;
import static frc.robot.Constants.ClimberConstants.CLIMB_POSITION;
import static frc.robot.Constants.ClimberConstants.CLIMB_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.EXTEND_POSITION_BACK;
import static frc.robot.Constants.ClimberConstants.EXTEND_POSITION_FRONT;
import static frc.robot.Constants.ClimberConstants.EXTEND_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.ClimberConstants.STALL_CURRENT;
import static frc.robot.Constants.ClimberConstants.STOW_POSITION;
import static frc.robot.Constants.ClimberConstants.STOW_SLOW_POSITION;
import static frc.robot.Constants.ClimberConstants.STOW_SLOW_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.STOW_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.ZERO_VOLTAGE;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final ClimberVisualizer visualizer = new ClimberVisualizer();

    private boolean disabled = false;

    public Climber(ClimberIO io) {
        this.io = io;

        SmartDashboard.putData("Climb/Climb", climb());
        SmartDashboard.putData("Climb/AutoClimb", autoClimb());
        SmartDashboard.putData("Climb/Stow", stow());
        SmartDashboard.putData("Climb/Extend", extend());
        SmartDashboard.putData("Climb/Zero", zero());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        visualizer.update(inputs.frontPosition, inputs.backPosition);
    }

    private Command setVoltage(Voltage out) {
        return this.runOnce(() -> io.setVoltage(out));
    }

    private Command stop() {
        return this.runOnce(io::stop);
    }

    public boolean isExtended() {
        return inputs.frontPosition.gte(EXTEND_POSITION_FRONT) && inputs.backPosition.gte(EXTEND_POSITION_BACK);
    }

    public Command climb() {
        return Commands.sequence(
                        setVoltage(CLIMB_VOLTAGE),
                        Commands.waitUntil(() -> inputs.averagePosition.lte(CLIMB_POSITION)),
                        setVoltage(Volts.of(-0.5)),
                        Commands.idle())
                .finallyDo(io::stop).unless(() -> disabled);
    }

    public Command autoClimb() {
        return Commands.sequence(
                        setVoltage(CLIMB_VOLTAGE),
                        Commands.waitUntil(() -> inputs.averagePosition.lte(AUTO_CLIMB_POSITION)),
                        setVoltage(Volts.of(-0.5)),
                        Commands.idle())
                .finallyDo(io::stop).unless(() -> disabled);
    }

    public Command stow() {
        return Commands.sequence(
                        setVoltage(STOW_VOLTAGE),
                        Commands.waitUntil(() -> inputs.frontPosition.lte(STOW_SLOW_POSITION)
                                || inputs.backPosition.lte(STOW_SLOW_POSITION)),
                        setVoltage(STOW_SLOW_VOLTAGE),
                        Commands.parallel(
                                Commands.waitUntil(() -> inputs.frontPosition.lte(STOW_POSITION))
                                        .finallyDo(io::stopFront),
                                Commands.waitUntil(() -> inputs.backPosition.lte(STOW_POSITION))
                                        .finallyDo(io::stopBack)),
                        stop())
                .finallyDo(io::stop).unless(() -> disabled);
    }

    public Command extend() {
        return Commands.sequence(
                        setVoltage(EXTEND_VOLTAGE),
                        Commands.parallel(
                                Commands.waitUntil(() -> inputs.frontPosition.gte(EXTEND_POSITION_FRONT))
                                        .finallyDo(io::stopFront),
                                Commands.waitUntil(() -> inputs.backPosition.gte(EXTEND_POSITION_BACK))
                                        .finallyDo(io::stopBack)),
                        stop())
                .finallyDo(io::stop).unless(() -> disabled);
    }

    public Command zero() {
        return Commands.sequence(
                        this.runOnce(() -> {
                            io.setFrontVoltage(ZERO_VOLTAGE);
                            io.setBackVoltage(ZERO_VOLTAGE);
                        }),
                        Commands.waitSeconds(0.1),
                        Commands.parallel(
                                Commands.waitUntil(() -> inputs.frontTorqueCurrent.abs(Amps) > STALL_CURRENT.abs(Amps)
                                                && inputs.frontVelocity.abs(RadiansPerSecond)
                                                        < STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond))
                                        .finallyDo(() -> io.stopFront()),
                                Commands.waitUntil(() -> inputs.backTorqueCurrent.abs(Amps) > STALL_CURRENT.abs(Amps)
                                                && inputs.backVelocity.abs(RadiansPerSecond)
                                                        < STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond))
                                        .finallyDo(() -> io.stopBack())),
                        Commands.waitSeconds(0.4),
                        this.runOnce(io::zeroPosition))
                .finallyDo(io::stop).unless(() -> disabled);
    }

    public Command disable() {
        return this.runOnce(() -> disabled = true).andThen(Commands.idle()).finallyDo(() -> disabled = false).withName("Disable Climber");
    }
}
