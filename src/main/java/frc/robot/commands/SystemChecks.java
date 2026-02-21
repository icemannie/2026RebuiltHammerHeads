package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.TurretConstants.MIN_HOOD_ANGLE;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intakes;
import frc.robot.subsystems.intake.Intakes.IntakesGoal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;

public class SystemChecks {
    private final Turret turret;
    private final Intakes intakes;
    private final Indexer indexer;
    private final Climber climber;

    public SystemChecks(Turret turret, Intakes intakes, Indexer indexer, Climber climber) {
        this.turret = turret;
        this.intakes = intakes;
        this.indexer = indexer;
        this.climber = climber;

        SmartDashboard.putData("System Checks/Turret", turret());
        SmartDashboard.putData("System Checks/Intakes", intakes());
        SmartDashboard.putData("System Checks/Indexer", indexer());
        SmartDashboard.putData("System Checks/Climber", climber());
    }

    public Command turret() {
        return Commands.sequence(turretTurn(), turretHood(), turretFlywheels());
    }

    public Command turretTurn() {
        return Commands.sequence(
                turret.setTurnPosition(Degrees.of(0)),
                Commands.waitSeconds(0.5),
                turret.setTurnPosition(Degrees.of(45)),
                Commands.waitSeconds(0.5),
                turret.setTurnPosition(Degrees.of(-45)),
                Commands.waitSeconds(0.5),
                turret.setTurnPosition(Degrees.of(0)));
    }

    public Command turretHood() {
        return Commands.sequence(
                turret.zeroHoodSequence(),
                Commands.waitSeconds(0.5),
                turret.setHoodPosition(Degrees.of(30)),
                Commands.waitSeconds(0.5),
                turret.setHoodPosition(MIN_HOOD_ANGLE));
    }

    public Command turretFlywheels() {
        return Commands.sequence(
                turret.setFlywheelSpeed(RPM.of(2500)), Commands.waitSeconds(3), turret.setGoal(TurretGoal.OFF));
    }

    public Command intakes() {
        return Commands.sequence(
                intakes.left.zeroSequence(),
                intakes.right.zeroSequence(),
                leftIntake(),
                rightIntake(),
                Commands.waitSeconds(0.5),
                intakes.setGoal(IntakesGoal.OFF));
    }

    public Command leftIntake() {
        return Commands.sequence(intakes.deployLeft(), Commands.waitSeconds(1), intakes.setGoal(IntakesGoal.STOW));
    }

    public Command rightIntake() {
        return Commands.sequence(intakes.deployRight(), Commands.waitSeconds(1), intakes.setGoal(IntakesGoal.STOW));
    }

    public Command indexer() {
        return Commands.sequence(indexer.activate(), Commands.waitSeconds(3), indexer.stop());
    }

    public Command climber() {
        return Commands.sequence(
                climber.zero(), Commands.waitSeconds(1), climber.extend(), Commands.waitSeconds(1), climber.stow());
    }
}
