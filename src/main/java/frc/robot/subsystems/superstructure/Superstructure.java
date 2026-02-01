// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.TurretConstants.PASSING_SPOT_LEFT;
import static frc.robot.Constants.TurretConstants.PASSING_SPOT_RIGHT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.HubTracker;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
    private final Turret turret;
    private final Intake intake;

    private final Supplier<Pose2d> poseSupplier;

    @AutoLogOutput
    private Goal goal = Goal.IDLE;

    private Goal nonCollectingGoal = goal;

    @AutoLogOutput
    public final Trigger inAllianceZoneTrigger = new Trigger(this::inAllianceZone);

    @AutoLogOutput
    public final Trigger activeHubTrigger = new Trigger(HubTracker::isActive).or(() -> HubTracker.getMatchTime() < 0);

    @AutoLogOutput
    public final Trigger onLeftSideTrigger;

    private final Map<Goal, Command> goalCommands;

    /** Creates a new Superstructure. */
    public Superstructure(Turret turret, Intake intake, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.intake = intake;
        this.poseSupplier = poseSupplier;

        onLeftSideTrigger = new Trigger(() -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && poseSupplier.get().getMeasureY().gt(FieldConstants.FIELD_WIDTH.div(2)))
                || (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        && poseSupplier.get().getMeasureY().lte(FieldConstants.FIELD_WIDTH.div(2))));

        goalCommands = Map.of(
                Goal.SCORING,
                Commands.sequence(
                                this.turret.setTarget(FieldConstants.HUB_BLUE),
                                this.turret.start(),
                                this.intake.setAutomaticDeploy(true))
                        .withName("Start scoring"),
                Goal.PASSING,
                Commands.sequence(
                                this.turret.setTarget(
                                        onLeftSideTrigger.getAsBoolean() ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT),
                                this.turret.start(),
                                this.intake.setAutomaticDeploy(true))
                        .withName("Start passing"),
                Goal.COLLECTING,
                Commands.sequence(
                                this.turret.stop(),
                                this.intake.setAutomaticDeploy(false),
                                Commands.either(
                                        this.intake.deployLeft(),
                                        this.intake.deployRight(),
                                        this.intake::travelingLeft))
                        .withName("Start collecting"),
                Goal.IDLE,
                Commands.sequence(this.turret.stop(), this.intake.stow()).withName("Idle"));

        inAllianceZoneTrigger.and(activeHubTrigger).onTrue(this.setGoal(Goal.SCORING));
        inAllianceZoneTrigger.and(activeHubTrigger.negate()).onTrue(this.setGoal(Goal.IDLE));
        inAllianceZoneTrigger.onFalse(this.setGoal(Goal.PASSING));

        onLeftSideTrigger
                .and(() -> this.goal == Goal.PASSING)
                .debounce(0.1)
                .onTrue(this.turret.setTarget(PASSING_SPOT_LEFT));
        onLeftSideTrigger
                .negate()
                .and(() -> this.goal == Goal.PASSING)
                .debounce(0.1)
                .onTrue(this.turret.setTarget(PASSING_SPOT_RIGHT));
    }

    private boolean inAllianceZone() {
        Pose2d pose = poseSupplier.get();
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && pose.getMeasureX().lt(FieldConstants.ALLIANCE_ZONE)
                || DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        && pose.getMeasureX().gt(FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE));
    }

    public Command setGoal(Goal goal) {
        return this.runOnce(() -> this.goal = goal).andThen(goalCommands.get(goal));
    }

    public Command startCollecting() {
        return this.setGoal(Goal.COLLECTING).beforeStarting(() -> nonCollectingGoal = goal);
    }

    public Command stopCollecting() {
        return this.setGoal(nonCollectingGoal);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public static enum Goal {
        SCORING,
        PASSING,
        COLLECTING,
        IDLE
    }
}
