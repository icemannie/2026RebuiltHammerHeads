package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.BACK_ID;
import static frc.robot.Constants.ClimberConstants.BACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.ClimberConstants.CURRENT_LIMITS_CONFIGS;
import static frc.robot.Constants.ClimberConstants.DIFF_KP;
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
    private final StatusSignal<Voltage> frontAppliedVoltage;

    private final StatusSignal<Angle> backPosition;
    private final StatusSignal<AngularVelocity> backVelocity;
    private final StatusSignal<Current> backTorqueCurrent;
    private final StatusSignal<Voltage> backAppliedVoltage;

    private final TalonFXConfiguration frontConfigs;
    private final TalonFXConfiguration backConfigs;

    private final VoltageOut frontVoltageOut = new VoltageOut(0);
    private final VoltageOut backVoltageOut = new VoltageOut(0);
    private final StaticBrake neutralOut = new StaticBrake();

    private boolean differentialEnabled = false;
    private Voltage frontRequestedVolts = Volts.of(0);
    private Voltage backRequestedVolts = Volts.of(0);

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
        frontAppliedVoltage = frontMotor.getMotorVoltage();

        backPosition = backMotor.getPosition();
        backVelocity = backMotor.getVelocity();
        backTorqueCurrent = backMotor.getTorqueCurrent();
        backAppliedVoltage = backMotor.getMotorVoltage();

        PhoenixUtil.registerStatusSignals(
                Hertz.of(50),
                frontPosition,
                frontVelocity,
                frontTorqueCurrent,
                frontAppliedVoltage,
                backPosition,
                backVelocity,
                backTorqueCurrent,
                backAppliedVoltage);

        frontMotor.optimizeBusUtilization();
        backMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.frontConnected =
                BaseStatusSignal.isAllGood(frontPosition, frontVelocity, frontTorqueCurrent, frontAppliedVoltage);
        inputs.frontPosition = frontPosition.getValue();
        inputs.frontVelocity = frontVelocity.getValue();
        inputs.frontTorqueCurrent = frontTorqueCurrent.getValue();
        inputs.frontAppliedVoltage = frontAppliedVoltage.getValue();

        inputs.backConnected =
                BaseStatusSignal.isAllGood(backPosition, backVelocity, backTorqueCurrent, backAppliedVoltage);
        inputs.backPosition = backPosition.getValue();
        inputs.backVelocity = backVelocity.getValue();
        inputs.backTorqueCurrent = backTorqueCurrent.getValue();
        inputs.backAppliedVoltage = backAppliedVoltage.getValue();

        inputs.averagePosition = inputs.frontPosition.plus(inputs.backPosition).div(2.0);

        if (differentialEnabled) {
            applyDifferential(inputs.frontPosition, inputs.backPosition);
        }
    }

    private void applyDifferential(Angle frontPos, Angle backPos) {
        double diff = frontPos.in(Radians) - backPos.in(Radians);
        frontMotor.setControl(frontVoltageOut.withOutput(frontRequestedVolts.minus(Volts.of(diff * 0.5 * DIFF_KP))));
        backMotor.setControl(backVoltageOut.withOutput(backRequestedVolts.plus(Volts.of(diff * 0.5 * DIFF_KP))));
    }

    @Override
    public void setVoltage(Voltage out) {
        frontRequestedVolts = out;
        backRequestedVolts = out;
        frontMotor.setControl(frontVoltageOut.withOutput(out));
        backMotor.setControl(backVoltageOut.withOutput(out));
        differentialEnabled = false;
    }

    @Override
    public void setFrontVoltage(Voltage out) {
        frontMotor.setControl(frontVoltageOut.withOutput(out));
        differentialEnabled = false;
    }

    @Override
    public void setBackVoltage(Voltage out) {
        backMotor.setControl(backVoltageOut.withOutput(out));
        differentialEnabled = false;
    }

    @Override
    public void stop() {
        frontMotor.setControl(neutralOut);
        backMotor.setControl(neutralOut);

        differentialEnabled = false;
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
