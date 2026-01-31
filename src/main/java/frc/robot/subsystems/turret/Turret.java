// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
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

    private final TurretVisualizer turretVisualizer;

    private final LoggedTunableNumber turnKP = new LoggedTunableNumber("Turret/Turn/kP", TURN_GAINS.kP);
    private final LoggedTunableNumber turnKD = new LoggedTunableNumber("Turret/Turn/kD", TURN_GAINS.kD);
    private final LoggedTunableNumber turnKV = new LoggedTunableNumber("Turret/Turn/kV", TURN_GAINS.kV);
    private final LoggedTunableNumber turnKS = new LoggedTunableNumber("Turret/Turn/kS", TURN_GAINS.kS);
    private final LoggedTunableNumber hoodKP = new LoggedTunableNumber("Turret/Hood/kP", HOOD_GAINS.kP);
    private final LoggedTunableNumber hoodKD = new LoggedTunableNumber("Turret/Hood/kD", HOOD_GAINS.kD);
    private final LoggedTunableNumber hoodKS = new LoggedTunableNumber("Turret/Hood/kS", HOOD_GAINS.kS);
    private final LoggedTunableNumber flywheelKP = new LoggedTunableNumber("Turret/Flywheel/kP", FLYWHEEL_GAINS.kP);
    private final LoggedTunableNumber flywheelKD = new LoggedTunableNumber("Turret/Flywheel/kD", FLYWHEEL_GAINS.kD);

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(inputs.turnPosition)))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                fieldSpeedsSupplier);

        if (Constants.CURRENT_MODE == Mode.SIM) {
            this.setDefaultCommand(turretVisualizer.repeatedlyLaunchFuel(
                    () -> TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS),
                    () -> inputs.hoodPosition,
                    this));

            SmartDashboard.putData(this.runOnce(() -> turretVisualizer.launchFuel(
                            TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS),
                            inputs.hoodPosition))
                    .withName("Launch Fuel"));
        }
    }

    public boolean simAbleToIntake() {
        return turretVisualizer.canIntake();
    }

    public void simIntake() {
        turretVisualizer.intakeFuel();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.HUB_BLUE
                : FieldConstants.HUB_RED;

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

        turretVisualizer.update3dPose(inputs.turnPosition);

        updateTunables();

        Logger.recordOutput("Turret/Shot", calculatedShot);
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

        if (flywheelKP.hasChanged(hashCode()) || flywheelKD.hasChanged(hashCode())) {
            io.setFlywheelPID(flywheelKP.get(), flywheelKD.get());
        }
    }

    @Override
    public void simulationPeriodic() {
        turretVisualizer.updateFuel(
                TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS), inputs.hoodPosition);
    }
}
