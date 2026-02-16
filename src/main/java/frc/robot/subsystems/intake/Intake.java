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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO leftIO;
    private final IntakeIO rightIO;
    private final IntakeIOInputsAUtoLogged leftInputs;
    private final IntakeIOInputsAutoLogged rightInputs;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    @AutoLogOutput
    private IntakeGoal goal = IntakeGoal.AUTOSWITCH;

    private boolean leftDeployed = false;
    private boolean rightDeployed = false;

    @AutoLogOutput
    public Trigger deployLeftTrigger = new Trigger(this::travelingLeft)
            .and(() -> goal == IntakeGoal.AUTOSWITCH)
            .debounce(0.08);

    @AutoLogOutput
    public Trigger deployRightTrigger = new Trigger(this::travelingRight)
            .and(() -> goal == IntakeGoal.AUTOSWITCH)
            .debounce(0.08);

    @AutoLogOutput
    private Trigger rightRackStallTrigger;

    @AutoLogOutput
    private Trigger leftRackStallTrigger;

    private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

    private final LoggedTunableNumber rackKP = new LoggedTunableNumber("Intake/kP", LEFT_RACK_GAINS.kP);
    private final LoggedTunableNumber rackKD = new LoggedTunableNumber("Intake/kD", LEFT_RACK_GAINS.kD);
    private final LoggedTunableNumber rackKV = new LoggedTunableNumber("Intake/kV", LEFT_RACK_GAINS.kV);
    private final LoggedTunableNumber rackKA = new LoggedTunableNumber("Intake/kA", LEFT_RACK_GAINS.kA);
    private final LoggedTunableNumber rackKS = new LoggedTunableNumber("Intake/kS", LEFT_RACK_GAINS.kS);
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

        this.deployLeftTrigger.onTrue(deployLeft());
        this.deployRightTrigger.onTrue(deployRight());
        rightRackStallTrigger = new Trigger(() -> rightIO.rackIsStalled()).debounce(0.1);
        leftRackStallTrigger = new Trigger(() -> leftIO.rackIsStalled()).debounce(0.1);
        SmartDashboard.putData(deployLeft());
        SmartDashboard.putData(deployRight());
        SmartDashboard.putData(setGoal(IntakeGoal.STOW));
    }

    public boolean travelingLeft() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond > MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public boolean travelingRight() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond < -MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    @AutoLogOutput
    public boolean isLeftStowed() {
        return leftInputs.rackPosition.isNear(STOW_POS, STOW_TOLERANCE);
    }

    @AutoLogOutput
    public boolean isRightStowed() {
        return rightInputs.rackPosition.isNear(STOW_POS, STOW_TOLERANCE);
    }

    public boolean isLeftDeployed() {
        return leftDeployed;
    }

    public boolean isRightDeployed() {
        return rightDeployed;
    }

    public Command setGoal(IntakeGoal goal) {
        return this.runOnce(() -> {
            this.goal = goal;
            switch (goal) {
                case AUTOSWITCH:
                    break;
                case MANUAL:
                    break;
                case STOW:
                    leftIO.setRackPosition(STOW_POS);
                    rightIO.setRackPosition(STOW_POS);

                    leftIO.stopSpin();
                    rightIO.stopSpin();
                    break;
            }
        });
    }

    public IntakeGoal getGoal() {
        return goal;
    }

    public Command deployLeft() {
        return this.runOnce(() -> {
                    rightDeployed = false;
                    rightIO.setRackPosition(STOW_POS);
                    rightIO.stopSpin();
                })
                .andThen(Commands.waitUntil(this::isRightStowed))
                .andThen(this.runOnce(() -> {
                    leftDeployed = true;
                    leftIO.setRackPosition(DEPLOY_POS);
                    leftIO.setSpinOutput(Volts.of(spinVoltage.get()));
                }))
                .withName("Deploy Left Intake");
    }

    public Command deployRight() {
        return this.runOnce(() -> {
                    leftDeployed = false;
                    leftIO.setRackPosition(STOW_POS);
                    leftIO.stopSpin();
                })
                .andThen(Commands.waitUntil(this::isLeftStowed))
                .andThen(this.runOnce(() -> {
                    rightDeployed = true;
                    rightIO.setRackPosition(DEPLOY_POS);
                    rightIO.setSpinOutput(Volts.of(spinVoltage.get()));
                }))
                .withName("Deploy Right Intake");
    }

    public Command zeroRightSequence() {
        return Commands.sequence(
                this.runOnce(() -> rightIO.setRackOutput(ZEROING_VOLTAGE)),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(rightRackStallTrigger::getAsBoolean),
                this.runOnce(rightIO::stopRack),
                Commands.waitSeconds(0.1),
                this.runOnce(() -> {
                    rightIO.zeroPosition();
                    rightIO.setRackPosition(STOW_POS);
                }));
    }

    public Command zeroLeftSequence() {
        return Commands.sequence(
                this.runOnce(() -> leftIO.setRackOutput(ZEROING_VOLTAGE)),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(leftRackStallTrigger::getAsBoolean),
                this.runOnce(leftIO::stopRack),
                Commands.waitSeconds(0.1),
                this.runOnce(() -> {
                    leftIO.zeroPosition();
                    leftIO.setRackPosition(STOW_POS);
                }));
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
            // rightIO.setRackPID(
            //         rackKP.get(),
            //         rackKD.get(),
            //         rackKV.get(),
            //         rackKA.get(),
            //         rackKS.get(),
            //         rackMaxVel.get(),
            //         rackMaxAcc.get());
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

    public enum IntakeGoal {
        AUTOSWITCH,
        MANUAL,
        STOW
    }
}
