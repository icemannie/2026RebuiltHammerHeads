package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Constants.ClimberConstants.BACK_ID;
import static frc.robot.Constants.ClimberConstants.BACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.ClimberConstants.CURRENT_LIMITS_CONFIGS;
import static frc.robot.Constants.ClimberConstants.FRONT_ID;
import static frc.robot.Constants.ClimberConstants.FRONT_OUTPUT_CONFIGS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX frontMotor;
    private final TalonFX backMotor;

    private final StatusSignal<Angle> frontPosition;
    private final StatusSignal<AngularVelocity> frontVelocity;
    private final StatusSignal<Current> frontTorqueCurrent;
    private final StatusSignal<Current> frontSupplyCurrent;
    private final StatusSignal<Voltage> frontAppliedVoltage;

    private final StatusSignal<Angle> backPosition;
    private final StatusSignal<AngularVelocity> backVelocity;
    private final StatusSignal<Current> backTorqueCurrent;
    private final StatusSignal<Current> backSupplyCurrent;
    private final StatusSignal<Voltage> backAppliedVoltage;

    private final TalonFXConfiguration frontConfigs;
    private final TalonFXConfiguration backConfigs;

    private final VoltageOut frontVoltageOut = new VoltageOut(0);
    private final VoltageOut backVoltageOut = new VoltageOut(0);
    private final StaticBrake neutralOut = new StaticBrake();

    public ClimberIOTalonFX() {
        frontMotor = new TalonFX(FRONT_ID, Constants.CAN_FD_BUS);
        backMotor = new TalonFX(BACK_ID, Constants.CAN_FD_BUS);

        frontConfigs = new TalonFXConfiguration()
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withMotorOutput(FRONT_OUTPUT_CONFIGS)
                .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.5));
        backConfigs = new TalonFXConfiguration()
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withMotorOutput(BACK_OUTPUT_CONFIGS)
                .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.5));

        PhoenixUtil.tryUntilOk(5, () -> frontMotor.getConfigurator().apply(frontConfigs, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> backMotor.getConfigurator().apply(backConfigs, 0.25));

        frontPosition = frontMotor.getPosition();
        frontVelocity = frontMotor.getVelocity();
        frontTorqueCurrent = frontMotor.getTorqueCurrent();
        frontSupplyCurrent = frontMotor.getSupplyCurrent();
        frontAppliedVoltage = frontMotor.getMotorVoltage();

        backPosition = backMotor.getPosition();
        backVelocity = backMotor.getVelocity();
        backTorqueCurrent = backMotor.getTorqueCurrent();
        backSupplyCurrent = backMotor.getSupplyCurrent();
        backAppliedVoltage = backMotor.getMotorVoltage();

        PhoenixUtil.registerStatusSignals(
                Hertz.of(50),
                frontPosition,
                frontVelocity,
                frontTorqueCurrent,
                frontSupplyCurrent,
                frontAppliedVoltage,
                backPosition,
                backVelocity,
                backTorqueCurrent,
                backSupplyCurrent,
                backAppliedVoltage);

        frontMotor.optimizeBusUtilization();
        backMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.frontConnected = BaseStatusSignal.isAllGood(
                frontPosition, frontVelocity, frontTorqueCurrent, frontSupplyCurrent, frontAppliedVoltage);
        inputs.frontPosition = frontPosition.getValue();
        inputs.frontVelocity = frontVelocity.getValue();
        inputs.frontTorqueCurrent = frontTorqueCurrent.getValue();
        inputs.frontSupplyCurrent = frontSupplyCurrent.getValue();
        inputs.frontAppliedVoltage = frontAppliedVoltage.getValue();

        inputs.backConnected = BaseStatusSignal.isAllGood(
                backPosition, backVelocity, backTorqueCurrent, backSupplyCurrent, backAppliedVoltage);
        inputs.backPosition = backPosition.getValue();
        inputs.backVelocity = backVelocity.getValue();
        inputs.backTorqueCurrent = backTorqueCurrent.getValue();
        inputs.backSupplyCurrent = backSupplyCurrent.getValue();
        inputs.backAppliedVoltage = backAppliedVoltage.getValue();

        inputs.averagePosition = inputs.frontPosition.plus(inputs.backPosition).div(2.0);
    }

    @Override
    public void setVoltage(Voltage out) {
        frontMotor.setControl(frontVoltageOut.withOutput(out));
        backMotor.setControl(backVoltageOut.withOutput(out));
    }

    @Override
    public void setFrontVoltage(Voltage out) {
        frontMotor.setControl(frontVoltageOut.withOutput(out));
    }

    @Override
    public void setBackVoltage(Voltage out) {
        backMotor.setControl(backVoltageOut.withOutput(out));
    }

    @Override
    public void stop() {
        frontMotor.setControl(neutralOut);
        backMotor.setControl(neutralOut);
    }

    @Override
    public void stopFront() {
        frontMotor.setControl(neutralOut);
    }

    @Override
    public void stopBack() {
        backMotor.setControl(neutralOut);
    }

    @Override
    public void zeroPosition() {
        frontMotor.setPosition(0);
        backMotor.setPosition(0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        frontMotor.setNeutralMode(neutralModeValue);
        backMotor.setNeutralMode(neutralModeValue);
    }
}
