// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intakes extends SubsystemBase {
    public final Intake left;
    public final Intake right;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    @AutoLogOutput
    private IntakesGoal goal = IntakesGoal.AUTOSWITCH;

    @AutoLogOutput
    public Trigger deployLeftTrigger = new Trigger(this::travelingLeft)
            .and(() -> goal == IntakesGoal.AUTOSWITCH)
            .debounce(0.08);

    @AutoLogOutput
    public Trigger deployRightTrigger = new Trigger(this::travelingRight)
            .and(() -> goal == IntakesGoal.AUTOSWITCH)
            .debounce(0.08);

    private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

    /** Creates a new Intake. */
    public Intakes(IntakeIO leftIO, IntakeIO rightIO, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.left = new Intake(leftIO, IntakeSide.LEFT);
        this.right = new Intake(rightIO, IntakeSide.RIGHT);

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.deployLeftTrigger.onTrue(deployLeft());
        this.deployRightTrigger.onTrue(deployRight());

        SmartDashboard.putData("Intakes/Deploy Left", left.deploy());
        SmartDashboard.putData("Intakes/Deploy Right", right.deploy());
        SmartDashboard.putData("Intakes/Stow Left", left.stow());
        SmartDashboard.putData("Intakes/Stow Right", right.stow());
        SmartDashboard.putData("Intakes/Autoswitch", setGoal(IntakesGoal.AUTOSWITCH));
        SmartDashboard.putData("Intakes/Stow All", setGoal(IntakesGoal.STOW));
        SmartDashboard.putData("Intakes/Disable", setGoal(IntakesGoal.OFF));
    }

    public boolean travelingLeft() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond > MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public boolean travelingRight() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond < -MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public Command setGoal(IntakesGoal goal) {
        return Commands.runOnce(() -> this.goal = goal)
                .andThen(Commands.select(
                        Map.of(
                                IntakesGoal.AUTOSWITCH,
                                Commands.none(),
                                IntakesGoal.MANUAL,
                                Commands.none(),
                                IntakesGoal.OFF,
                                left.off().alongWith(right.off()),
                                IntakesGoal.STOW,
                                left.stow().alongWith(right.stow())),
                        () -> goal));
    }

    public IntakesGoal getGoal() {
        return goal;
    }

    public Command deployLeft() {
        return Commands.sequence(right.stow(), Commands.waitUntil(right.stowedTrigger), left.deploy());
    }

    public Command deployRight() {
        return Commands.sequence(left.stow(), Commands.waitUntil(left.stowedTrigger), right.deploy());
    }

    public Command switchIntakes() {
        return Commands.either(
                deployLeft(),
                deployRight(),
                () -> (right.getGoal() == IntakeGoal.DEPLOYED || right.getGoal() == IntakeGoal.DEPLOYING));
    }

    @Override
    public void periodic() {
        measuredVisualizer.setLeftPosition(left.getPosition());
        measuredVisualizer.setRightPosition(right.getPosition());
    }

    public enum IntakesGoal {
        AUTOSWITCH,
        MANUAL,
        STOW,
        OFF
    }

    public enum IntakeSide {
        LEFT,
        RIGHT
    }
}
