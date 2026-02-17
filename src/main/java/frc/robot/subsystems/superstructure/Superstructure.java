// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intakes;
import frc.robot.subsystems.intake.Intakes.IntakesGoal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;
import frc.robot.util.HubTracker;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
    private final Turret turret;
    private final Intakes intake;
    private final Indexer indexer;

    private final Supplier<Pose2d> poseSupplier;

    @AutoLogOutput
    private Goal goal = Goal.PASSING;

    private Goal nonCollectingGoal = goal;

    @AutoLogOutput
    public final Trigger inAllianceZoneTrigger = new Trigger(this::inAllianceZone);

    @AutoLogOutput
    public final Trigger activeHubTrigger = new Trigger(HubTracker::isActive).or(() -> HubTracker.getMatchTime() < 0);

    @AutoLogOutput
    public final Trigger onLeftSideTrigger;

    private final Map<Goal, Command> goalCommands;

    /** Creates a new Superstructure. */
    public Superstructure(Turret turret, Intakes intake, Indexer indexer, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.intake = intake;
        this.indexer = indexer;
        this.poseSupplier = poseSupplier;

        onLeftSideTrigger = new Trigger(() -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && poseSupplier.get().getMeasureY().gt(FieldConstants.FIELD_WIDTH.div(2)))
                || (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        && poseSupplier.get().getMeasureY().lte(FieldConstants.FIELD_WIDTH.div(2))));

        goalCommands = Map.of(
                Goal.SCORING,
                Commands.sequence(
                                this.turret.setGoal(TurretGoal.SCORING),
                                this.intake.setGoal(IntakesGoal.AUTOSWITCH),
                                this.indexer.setGoal(IndexerGoal.ACTIVE))
                        .withName("Start scoring"),
                Goal.PASSING,
                Commands.sequence(
                                this.turret.setGoal(TurretGoal.PASSING),
                                this.intake.setGoal(IntakesGoal.AUTOSWITCH),
                                this.indexer.setGoal(IndexerGoal.ACTIVE))
                        .withName("Start passing"),
                Goal.COLLECTING,
                Commands.sequence(
                                this.turret.setGoal(TurretGoal.IDLE),
                                this.intake.setGoal(IntakesGoal.MANUAL),
                                this.indexer.setGoal(IndexerGoal.OFF),
                                Commands.either(
                                        this.intake.deployLeft(),
                                        this.intake.deployRight(),
                                        this.intake::travelingLeft))
                        .withName("Start collecting"),
                Goal.IDLE,
                Commands.sequence(
                                this.turret.setGoal(TurretGoal.IDLE),
                                this.intake.setGoal(IntakesGoal.STOW),
                                this.indexer.setGoal(IndexerGoal.OFF))
                        .withName("Idle"));

        inAllianceZoneTrigger.onTrue(this.setGoal(Goal.SCORING));
        // inAllianceZoneTrigger.and(activeHubTrigger.negate()).onTrue(this.setGoal(Goal.IDLE));
        inAllianceZoneTrigger.onFalse(this.setGoal(Goal.PASSING));
    }

    private boolean inAllianceZone() {
        Pose2d pose = poseSupplier.get();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue && pose.getMeasureX().lt(FieldConstants.ALLIANCE_ZONE)
                || !isBlue && pose.getMeasureX().gt(FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE));
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
