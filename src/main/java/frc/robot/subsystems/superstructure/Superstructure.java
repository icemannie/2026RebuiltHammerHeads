// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.HubTracker;

import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    private final Turret turret;
    private final Intake intake;

    private final Supplier<Pose2d> poseSupplier;

    private Goal goal = Goal.IDLE;

    public final Trigger inAllianceZoneTrigger = new Trigger(this::inAllianceZone);
    public final Trigger activeHubTrigger = new Trigger(HubTracker::isActive);

    /** Creates a new Superstructure. */
    public Superstructure(Turret turret, Intake intake, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.intake = intake;
        this.poseSupplier = poseSupplier;

        inAllianceZoneTrigger.and(activeHubTrigger).onTrue(this.setGoal(Goal.SCORING));
    }

    private boolean inAllianceZone() {
        Pose2d pose = poseSupplier.get();
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && pose.getMeasureX().lt(FieldConstants.ALLIANCE_ZONE)
                || DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && pose.getMeasureX().gt(FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE));
    }

    public Command setGoal(Goal goal) {
        return this.runOnce(() -> {
            this.goal = goal;
            switch (goal) {
                case SCORING:
                    turret.setTarget(FieldConstants.HUB_BLUE);
                    intake.setAutomaticDeploy(true);
                    break;

                default:
                    break;
            }
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public static enum Goal {
        SCORING,
        DUMPING,
        PASSING,
        COLLECTING,
        IDLE
    }
}
