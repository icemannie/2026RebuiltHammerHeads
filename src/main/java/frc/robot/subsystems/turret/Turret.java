// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.IntakeConstants.ZEROING_VOLTAGE;
import static frc.robot.Constants.TurretConstants.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    @AutoLogOutput
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;

    private boolean isActive = false;

    private final TurretVisualizer turretVisualizer;

    private final Trigger hoodStalledTrigger;

    private final LoggedTunableNumber turnKP = new LoggedTunableNumber("Turret/Turn/kP", TURN_GAINS.kP);
    private final LoggedTunableNumber turnKD = new LoggedTunableNumber("Turret/Turn/kD", TURN_GAINS.kD);
    private final LoggedTunableNumber turnKV = new LoggedTunableNumber("Turret/Turn/kV", TURN_GAINS.kV);
    private final LoggedTunableNumber turnKS = new LoggedTunableNumber("Turret/Turn/kS", TURN_GAINS.kS);
    private final LoggedTunableNumber hoodKP = new LoggedTunableNumber("Turret/Hood/kP", HOOD_GAINS.kP);
    private final LoggedTunableNumber hoodKD = new LoggedTunableNumber("Turret/Hood/kD", HOOD_GAINS.kD);
    private final LoggedTunableNumber hoodKS = new LoggedTunableNumber("Turret/Hood/kS", HOOD_GAINS.kS);
    private final LoggedTunableNumber flywheelKP = new LoggedTunableNumber("Turret/Flywheel/kP", FLYWHEEL_GAINS.kP);
    private final LoggedTunableNumber flywheelKD = new LoggedTunableNumber("Turret/Flywheel/kD", FLYWHEEL_GAINS.kD);
    private final LoggedTunableNumber flywheelKV = new LoggedTunableNumber("Turret/Flywheel/kV", FLYWHEEL_GAINS.kV);
    private final LoggedTunableNumber flywheelKS = new LoggedTunableNumber("Turret/Flywheel/kS", FLYWHEEL_GAINS.kS);

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        io.zeroHoodPosition();

        hoodStalledTrigger = new Trigger(() -> inputs.hoodCurrent.abs(Amps) >= HOOD_STALL_CURRENT.abs(Amps)
                && inputs.hoodVelocity.abs(RadiansPerSecond) <= HOOD_STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond));

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(inputs.turnPosition)))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                fieldSpeedsSupplier);
    }

    public Command stop() {
        return this.runOnce(() -> {
            io.stopFlywheel();
            io.stopHood();
            io.stopTurn();
            isActive = false;
        });
    }

    public Command setHoodPosition(Angle angle) {
        return this.runOnce(() -> io.setHoodAngle(angle));
    }

    public Command setFlywheelSpeed(AngularVelocity speed) {
        return this.runOnce(() -> io.setFlywheelSpeed(speed));
    }

    public Command start() {
        return this.runOnce(() -> isActive = true);
    }

    public Command setTarget(Translation3d target) {
        return this.runOnce(() -> {
            currentTarget = target;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
                currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
            }
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (isActive) {
            calculateShot();
        }

        turretVisualizer.update3dPose(inputs.turnPosition);
        updateTunables();
    }

    private void calculateShot() {
        Pose2d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                robot, fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
        Angle azimuthAngle = TurretCalculator.calculateAzimuthAngle(robot, calculatedShot.target());
        AngularVelocity azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        io.setTurnSetpoint(azimuthAngle, azimuthVelocity);
        io.setHoodAngle(calculatedShot.getHoodAngle());
        io.setFlywheelSpeed(
                TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), FLYWHEEL_RADIUS));

        Logger.recordOutput("Turret/Shot", calculatedShot);
    }

    public Command zeroHoodSequence() {
        return Commands.sequence(
                this.runOnce(() -> io.setHoodOut(ZEROING_VOLTAGE)),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(hoodStalledTrigger::getAsBoolean),
                this.runOnce(io::stopHood),
                Commands.waitSeconds(0.1),
                this.runOnce(() -> {
                    io.zeroHoodPosition();
                    io.setHoodAngle(MIN_HOOD_ANGLE);
                }));
    }

    private void updateTunables() {
        if (turnKP.hasChanged(hashCode())
                || turnKD.hasChanged(hashCode())
                || turnKV.hasChanged(hashCode())
                || turnKS.hasChanged(hashCode())) {
            io.setTurnPID(turnKP.get(), turnKD.get(), turnKV.get(), turnKS.get());
        }

        if (hoodKP.hasChanged(hashCode()) || hoodKD.hasChanged(hashCode()) || hoodKS.hasChanged(hashCode())) {
            io.setHoodPID(hoodKP.get(), hoodKD.get(), hoodKS.get());
        }

        if (flywheelKP.hasChanged(hashCode())
                || flywheelKD.hasChanged(hashCode())
                || flywheelKV.hasChanged(hashCode())
                || flywheelKS.hasChanged(hashCode())) {
            io.setFlywheelPID(flywheelKP.get(), flywheelKD.get(), flywheelKV.get(), flywheelKS.get());
        }
    }

    @Override
    public void simulationPeriodic() {
        turretVisualizer.updateFuel(
                TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS), inputs.hoodPosition);
    }
}
