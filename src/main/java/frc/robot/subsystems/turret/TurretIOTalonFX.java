// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.CAN_FD_BUS;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turnMotor;
    private final TalonFX hoodMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelFollowerMotor;

    private final TalonFXConfiguration turnConfig;
    private final TalonFXConfiguration hoodConfig;
    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration flywheelFollowerConfig;

    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodCurrent;

    private final StatusSignal<AngularVelocity> flywheelSpeed;
    private final StatusSignal<AngularAcceleration> flywheelAccel;
    private final StatusSignal<Double> flywheelSetpointSpeed;
    private final StatusSignal<Double> flywheelSetpointAccel;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;

    private final PositionVoltage turnPositionRequest = new PositionVoltage(0);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC flywheelTorqueRequest = new TorqueCurrentFOC(0);

    private final Follower followRequest = new Follower(
            FLYWHEEL_ID,
            FLYWHEEL_OUTPUT_CONFIGS.Inverted == FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS.Inverted
                    ? MotorAlignmentValue.Aligned
                    : MotorAlignmentValue.Opposed);
    private final NeutralOut neutralOut = new NeutralOut();
    private final BangBangController bangBangController =
            new BangBangController(RPM.of(100).in(RadiansPerSecond));

    public TurretIOTalonFX() {
        turnMotor = new TalonFX(TURN_ID, CAN_FD_BUS);
        hoodMotor = new TalonFX(HOOD_ID, CAN_FD_BUS);
        flywheelMotor = new TalonFX(FLYWHEEL_ID, CAN_FD_BUS);
        flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_ID, CAN_FD_BUS);

        turnConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackRemoteSensorID(ENCODER_ID)
                        .withSensorToMechanismRatio(ENCODER_TO_TURRET_RATIO)
                        .withRotorToSensorRatio(TURN_TO_TURRET_RATIO / ENCODER_TO_TURRET_RATIO)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder))
                .withSlot0(TURN_GAINS)
                .withCurrentLimits(TURN_CURRENT_LIMITS)
                .withMotorOutput(TURN_OUTPUT_CONFIGS)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(MAX_TURN_ANGLE)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(MIN_TURN_ANGLE));

        hoodConfig = new TalonFXConfiguration()
                .withSlot0(HOOD_GAINS)
                .withCurrentLimits(HOOD_CURRENT_LIMITS)
                .withMotorOutput(HOOD_OUTPUT_CONFIGS)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(MAX_HOOD_ANGLE)
                        .withReverseSoftLimitEnable(false)
                        .withReverseSoftLimitThreshold(MIN_HOOD_ANGLE))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_MOTOR_RATIO));

        flywheelConfig = new TalonFXConfiguration()
                .withSlot0(FLYWHEEL_GAINS)
                .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FLYWHEEL_OUTPUT_CONFIGS)
                .withFeedback(FLYWHEEL_FEEDBACK_CONFIGS);

        flywheelFollowerConfig = new TalonFXConfiguration()
                .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig, 0.25));

        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnCurrent = turnMotor.getStatorCurrent();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();

        flywheelSpeed = flywheelMotor.getVelocity();
        flywheelAccel = flywheelMotor.getAcceleration();
        flywheelSetpointSpeed = flywheelMotor.getClosedLoopReference();
        flywheelSetpointAccel = flywheelMotor.getClosedLoopReferenceSlope();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent,
                hoodPosition,
                hoodVelocity,
                hoodAppliedVolts,
                hoodCurrent,
                flywheelSpeed,
                flywheelAccel,
                flywheelSetpointSpeed,
                flywheelSetpointAccel,
                flywheelAppliedVolts,
                flywheelCurrent);
        turnMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
        flywheelFollowerMotor.optimizeBusUtilization();

        flywheelFollowerMotor.setControl(followRequest);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
                        turnPosition, turnVelocity, turnAppliedVolts, turnCurrent)
                .isOK();
        inputs.turnPosition = turnPosition.getValue();
        inputs.turnVelocity = turnVelocity.getValue();
        inputs.turnAppliedVolts = turnAppliedVolts.getValue();
        inputs.turnCurrent = turnCurrent.getValue();

        inputs.hoodMotorConnected = BaseStatusSignal.refreshAll(
                        hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent)
                .isOK();
        inputs.hoodPosition = hoodPosition.getValue();
        inputs.hoodVelocity = hoodVelocity.getValue();
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValue();
        inputs.hoodCurrent = hoodCurrent.getValue();

        inputs.flywheelMotorConnected = BaseStatusSignal.refreshAll(
                        flywheelSpeed,
                        flywheelAccel,
                        flywheelSetpointSpeed,
                        flywheelSetpointAccel,
                        flywheelAppliedVolts,
                        flywheelCurrent)
                .isOK();
        inputs.flywheelSpeed = flywheelSpeed.getValue();
        inputs.flywheelAccel = flywheelAccel.getValue();
        inputs.flywheelSetpointSpeed = RotationsPerSecond.of(flywheelSetpointSpeed.getValueAsDouble());
        inputs.flywheelSetpointAccel = RotationsPerSecondPerSecond.of(flywheelSetpointAccel.getValueAsDouble());
        inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
        if (flywheelMotor.getAppliedControl().getClass() == TorqueCurrentFOC.class) {
            flywheelMotor.setControl(flywheelTorqueRequest.withOutput(
                    BANG_BANG_AMPS.times(bangBangController.calculate(inputs.flywheelSpeed.in(RadiansPerSecond)))));
        }
    }

    @Override
    public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
        turnMotor.setControl(turnPositionRequest.withPosition(position).withVelocity(velocity));
    }

    @Override
    public void setHoodAngle(Angle angle) {
        hoodMotor.setControl(hoodPositionRequest.withPosition(angle));
    }

    @Override
    public void setHoodOut(Voltage out) {
        hoodMotor.setControl(hoodVoltageRequest.withOutput(out));
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity speed) {
        // flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(speed));
        bangBangController.setSetpoint(speed.in(RadiansPerSecond));
        flywheelMotor.setControl(flywheelTorqueRequest.withOutput(BANG_BANG_AMPS.times(
                bangBangController.calculate(flywheelSpeed.getValue().in(RadiansPerSecond)))));
    }

    @Override
    public void stopTurn() {
        turnMotor.setControl(neutralOut);
    }

    @Override
    public void stopHood() {
        hoodMotor.setControl(neutralOut);
    }

    @Override
    public void stopFlywheel() {
        flywheelMotor.setControl(neutralOut);
    }

    @Override
    public void resetTurnEncoder() {
        turnMotor.setPosition(turnPosition.getValue().in(Rotations) % 1);
    }

    @Override
    public void zeroHoodPosition() {
        hoodMotor.setPosition(MIN_HOOD_ANGLE);
    }

    @Override
    public void setTurnPID(double kP, double kD, double kV, double kS) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kD = kD;
        turnConfig.Slot0.kV = kV;
        turnConfig.Slot0.kS = kS;

        PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig.Slot0, 0.25));
    }

    @Override
    public void setHoodPID(double kP, double kD, double kS) {
        hoodConfig.Slot0.kP = kP;
        hoodConfig.Slot0.kD = kD;
        hoodConfig.Slot0.kS = kS;

        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig.Slot0, 0.25));
    }

    @Override
    public void setFlywheelPID(double kP, double kD, double kV, double kS) {
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kD = kD;
        flywheelConfig.Slot0.kV = kV;
        flywheelConfig.Slot0.kS = kS;

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig.Slot0, 0.25));
    }
}
