// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO leftIO;
    private final IntakeIO rightIO;
    private final IntakeIOInputsAutoLogged leftInputs;
    private final IntakeIOInputsAutoLogged rightInputs;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    @AutoLogOutput
    private boolean automaticDeploy = true;

    private boolean leftDeployed = false;
    private boolean rightDeployed = false;

    @AutoLogOutput
    public Trigger deployLeftTrigger =
            new Trigger(this::travelingLeft).and(() -> automaticDeploy).debounce(0.2);

    @AutoLogOutput
    public Trigger deployRightTrigger =
            new Trigger(this::travelingRight).and(() -> automaticDeploy).debounce(0.2);

    private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

    private final LoggedTunableNumber rackKP = new LoggedTunableNumber("Intake/kP", RACK_GAINS.kP);
    private final LoggedTunableNumber rackKD = new LoggedTunableNumber("Intake/kD", RACK_GAINS.kD);
    private final LoggedTunableNumber rackKV = new LoggedTunableNumber("Intake/kV", RACK_GAINS.kV);
    private final LoggedTunableNumber rackKA = new LoggedTunableNumber("Intake/kA", RACK_GAINS.kA);
    private final LoggedTunableNumber rackKS = new LoggedTunableNumber("Intake/kS", RACK_GAINS.kS);
    private final LoggedTunableNumber rackMaxVel =
            new LoggedTunableNumber("Intake/maxVelRotPerSec", RACK_MOTION_MAGIC.MotionMagicCruiseVelocity);
    private final LoggedTunableNumber rackMaxAcc =
            new LoggedTunableNumber("Intake/maxAccRotPerSecPerSec", RACK_MOTION_MAGIC.MotionMagicAcceleration);
    private final LoggedTunableNumber spinVoltage =
            new LoggedTunableNumber("Intake/Spin Voltage", SPIN_VOLTAGE.in(Volts));

    /** Creates a new Intake. */
    public Intake(IntakeIO leftIO, IntakeIO rightIO, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.leftIO = leftIO;
        this.rightIO = rightIO;

        this.leftInputs = new IntakeIOInputsAutoLogged();
        this.rightInputs = new IntakeIOInputsAutoLogged();

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        // this.deployLeftTrigger.onTrue(deployLeft());
        // this.deployRightTrigger.onTrue(deployRight());

        SmartDashboard.putData(deployLeft());
        SmartDashboard.putData(deployRight());
        SmartDashboard.putData(stow());
    }

    public boolean travelingLeft() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond > MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public boolean travelingRight() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond < -MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public boolean isLeftDeployed() {
        return leftDeployed;
    }

    public boolean isRightDeployed() {
        return rightDeployed;
    }

    public Command setAutomaticDeploy(boolean shouldDeployAutomatically) {
        return this.runOnce(() -> this.automaticDeploy = shouldDeployAutomatically);
    }

    public Command deploy() {
        return this.runOnce(() -> automaticDeploy = true)
                .andThen(deployLeft().onlyIf(() -> !travelingLeft() && !travelingRight()))
                .withName("Deploy intake");
    }

    public Command deployLeft() {
        return this.runOnce(() -> {
                    leftDeployed = true;
                    rightDeployed = false;
                    leftIO.setRackPosition(DEPLOY_POS);
                    rightIO.setRackPosition(STOW_POS);

                    leftIO.setSpinOutput(Volts.of(spinVoltage.get()));
                    rightIO.stopSpin();
                })
                .withName("Deploy Left Intake");
    }

    public Command deployRight() {
        return this.runOnce(() -> {
                    leftDeployed = false;
                    rightDeployed = true;
                    leftIO.setRackPosition(STOW_POS);
                    rightIO.setRackPosition(DEPLOY_POS);

                    leftIO.stopSpin();
                    rightIO.setSpinOutput(Volts.of(spinVoltage.get()));
                })
                .withName("Deploy Right Intake");
    }

    public Command stow() {
        return this.runOnce(() -> {
                    automaticDeploy = false;
                    leftIO.setRackPosition(STOW_POS);
                    rightIO.setRackPosition(STOW_POS);

                    leftIO.stopSpin();
                    rightIO.stopSpin();
                })
                .withName("Stow intakes");
    }

    private void updateTunables() {
        if (rackKP.hasChanged(hashCode())
                || rackKD.hasChanged(hashCode())
                || rackKV.hasChanged(hashCode())
                || rackKA.hasChanged(hashCode())
                || rackKS.hasChanged(hashCode())
                || rackMaxVel.hasChanged(hashCode())
                || rackMaxAcc.hasChanged(hashCode())) {
            leftIO.setRackPID(
                    rackKP.get(),
                    rackKD.get(),
                    rackKV.get(),
                    rackKA.get(),
                    rackKS.get(),
                    rackMaxVel.get(),
                    rackMaxAcc.get());
            rightIO.setRackPID(
                    rackKP.get(),
                    rackKD.get(),
                    rackKV.get(),
                    rackKA.get(),
                    rackKS.get(),
                    rackMaxVel.get(),
                    rackMaxAcc.get());
        }

        if (spinVoltage.hasChanged(hashCode())) {
            if (leftDeployed) {
                leftIO.setSpinOutput(Volts.of(spinVoltage.get()));
            }
            if (rightDeployed) {
                rightIO.setSpinOutput(Volts.of(spinVoltage.get()));
            }
        }
    }

    @Override
    public void periodic() {
        leftIO.updateInputs(leftInputs);
        rightIO.updateInputs(rightInputs);

        Logger.processInputs("Left Intake", leftInputs);
        Logger.processInputs("Right Intake", rightInputs);

        measuredVisualizer.setLeftPosition(leftInputs.rackPosition);
        measuredVisualizer.setRightPosition(rightInputs.rackPosition);

        updateTunables();
    }
}
