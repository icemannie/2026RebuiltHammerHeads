// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TurretConstants.FLYWHEEL_RADIUS;
import static frc.robot.Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.FuelSim;

/** Add your docs here. */
public class TurretIOSim implements TurretIO {
    private Angle turnPosition = Radians.zero();
    private Angle hoodAngle = Radians.zero();
    private AngularVelocity flywheelGoal = RadiansPerSecond.zero();

    private SlewRateLimiter flywheelAccelLimiter =
            new SlewRateLimiter(RPM.of(4000).div(Seconds.of(2)).in(RadiansPerSecondPerSecond));

    private final int CAPACITY = 30;
    private final FuelSim fuelSim;
    private int fuelStored = 8;

    private Timer shootTimer = new Timer();

    public TurretIOSim(FuelSim fuelSim) {
        this.fuelSim = fuelSim;
        shootTimer.start();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turnPosition = turnPosition;
        inputs.hoodPosition = hoodAngle;
        inputs.flywheelSpeed = RadiansPerSecond.of(flywheelAccelLimiter.calculate(flywheelGoal.in(RadiansPerSecond)));

        if (shootTimer.advanceIfElapsed(0.1) && DriverStation.isEnabled()) {
            launchFuel();
        }
    }

    @Override
    public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
        turnPosition = position;
    }

    @Override
    public void setHoodAngle(Angle angle) {
        hoodAngle = angle;
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity speed) {
        flywheelGoal = speed;
    }

    @Override
    public void stopFlywheel() {
        flywheelGoal = RadiansPerSecond.zero();
    }

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel() {
        if (fuelStored == 0) return;
        fuelStored--;

        fuelSim.launchFuel(
                TurretCalculator.angularToLinearVelocity(flywheelGoal, FLYWHEEL_RADIUS),
                Degrees.of(90).minus(hoodAngle),
                turnPosition,
                ROBOT_TO_TURRET_TRANSFORM.getMeasureZ());
    }
}
