package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Constants.ClimberConstants.BACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.ClimberConstants.CURRENT_LIMITS_CONFIGS;
import static frc.robot.Constants.ClimberConstants.FRONT_ID;
import static frc.robot.Constants.ClimberConstants.FRONT_OUTPUT_CONFIGS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class HookIOTalonFX implements HookIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Voltage> appliedVoltage;

    private final TalonFXConfiguration configs;

    private final VoltageOut voltageOut = new VoltageOut(0);

    public HookIOTalonFX(int motorID) {
        motor = new TalonFX(motorID, Constants.CAN_FD_BUS);

        configs = new TalonFXConfiguration().withCurrentLimits(CURRENT_LIMITS_CONFIGS);

        if (motorID == FRONT_ID) {
            configs.withMotorOutput(FRONT_OUTPUT_CONFIGS);
        } else {
            configs.withMotorOutput(BACK_OUTPUT_CONFIGS);
        }

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(configs, 0.25));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        torqueCurrent = motor.getTorqueCurrent();
        appliedVoltage = motor.getMotorVoltage();

        PhoenixUtil.registerStatusSignals(Hertz.of(50), position, velocity, torqueCurrent, appliedVoltage);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HookIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, torqueCurrent, appliedVoltage);
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.appliedVoltage = appliedVoltage.getValue();
    }

    @Override
    public void setVoltage(Voltage out) {
        motor.setControl(voltageOut.withOutput(out));
    }

    @Override
    public void zeroPosition() {
        motor.setPosition(0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        motor.setNeutralMode(neutralModeValue);
    }
}
