package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.FEED_CURRENT_LIMITS;
import static frc.robot.Constants.IndexerConstants.FEED_OUTPUT_CONFIGS;
import static frc.robot.Constants.IndexerConstants.SPIN_CURRENT_LIMITS;
import static frc.robot.Constants.IndexerConstants.SPIN_OUTPUT_CONFIGS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX spinMotor;
    private final TalonFX feedMotor;

    private final VoltageOut spinVoltageOut = new VoltageOut(0);
    private final VoltageOut feedVoltageOut = new VoltageOut(0);

    private final NeutralOut neutralOut = new NeutralOut();

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Voltage> spinAppliedVolts;

    private final StatusSignal<AngularVelocity> feedVelocity;
    private final StatusSignal<Current> feedCurrent;
    private final StatusSignal<Voltage> feedAppliedVolts;

    public IndexerIOTalonFX(int spinMotorID, int feedMotorID) {
        spinMotor = new TalonFX(spinMotorID, Constants.CAN_FD_BUS);
        feedMotor = new TalonFX(feedMotorID, Constants.CAN_FD_BUS);

        TalonFXConfiguration spinConfig =
                new TalonFXConfiguration().withMotorOutput(SPIN_OUTPUT_CONFIGS).withCurrentLimits(SPIN_CURRENT_LIMITS);

        PhoenixUtil.tryUntilOk(5, () -> spinMotor.getConfigurator().apply(spinConfig, 0.25));

        TalonFXConfiguration feedConfig =
                new TalonFXConfiguration().withMotorOutput(FEED_OUTPUT_CONFIGS).withCurrentLimits(FEED_CURRENT_LIMITS);

        PhoenixUtil.tryUntilOk(5, () -> feedMotor.getConfigurator().apply(feedConfig, 0.25));

        spinVelocity = spinMotor.getVelocity();
        spinCurrent = spinMotor.getStatorCurrent();
        spinAppliedVolts = spinMotor.getMotorVoltage();

        feedVelocity = feedMotor.getVelocity();
        feedCurrent = feedMotor.getStatorCurrent();
        feedAppliedVolts = feedMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50, spinVelocity, spinCurrent, spinAppliedVolts, feedVelocity, feedCurrent, feedAppliedVolts);

        spinMotor.optimizeBusUtilization();
        feedMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spinMotorConnected = BaseStatusSignal.refreshAll(spinVelocity, spinCurrent, spinAppliedVolts).isOK();
        inputs.spinVelocity = spinVelocity.getValue();
        inputs.spinCurrent = spinCurrent.getValue();
        inputs.spinAppliedVolts = spinAppliedVolts.getValue();

        inputs.feedMotorConnected = BaseStatusSignal.refreshAll(feedVelocity, feedCurrent, feedAppliedVolts).isOK();
        inputs.feedVelocity = feedVelocity.getValue();
        inputs.feedCurrent = feedCurrent.getValue();
        inputs.feedAppliedVolts = feedAppliedVolts.getValue();
    }

    @Override
    public void setSpinOutput(Voltage out) {
        spinMotor.setControl(spinVoltageOut.withOutput(out));
    }

    @Override
    public void setFeedOutput(Voltage out) {
        feedMotor.setControl(feedVoltageOut.withOutput(out));
    }

    @Override
    public void stopSpin() {
        spinMotor.setControl(neutralOut);
    }

    @Override
    public void stopFeed() {
        feedMotor.setControl(neutralOut);
    }
}
