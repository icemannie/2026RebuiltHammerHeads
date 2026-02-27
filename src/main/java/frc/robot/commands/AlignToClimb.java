// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ClimberConstants.CLIMB_ALIGN_CONSTANTS_ROTATION;
import static frc.robot.Constants.ClimberConstants.CLIMB_ALIGN_CONSTANTS_TRANSLATION;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableControls.TunablePIDController;
import org.littletonrobotics.junction.Logger;

/** Command to align to a given climb position (front/back + left/right) */
public class AlignToClimb extends Command {
    private final TunablePIDController translationController =
            new TunablePIDController(CLIMB_ALIGN_CONSTANTS_TRANSLATION);
    private final TunablePIDController rotationController = new TunablePIDController(CLIMB_ALIGN_CONSTANTS_ROTATION);

    private Pose2d targetPose;
    private final ClimbPosition position;

    private final Drive drive;

    /**
     * Creates a new AlignToClimb command
     * @param position which climb position to align to
     * @param drive drive subsystem
     */
    public AlignToClimb(ClimbPosition position, Drive drive) {
        this.position = position;
        this.drive = drive;
        this.targetPose = position.getPose();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            this.targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // find  target pose from climb position, flipping if necessary
        this.targetPose = position.getPose();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            this.targetPose = FlippingUtil.flipFieldPose(targetPose);
        }

        // reset PID controllers
        translationController.setSetpoint(0);
        translationController.reset();

        rotationController.setSetpoint(targetPose.getRotation().getRadians());
        rotationController.reset();

        Logger.recordOutput("Align/Target", targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // determine direction to move in
        Pose2d pose = drive.getPose();
        Translation2d direction = targetPose.getTranslation().minus(pose.getTranslation());
        direction = direction.div(direction.getNorm());

        // apply PID control along direction vector
        double translationOutput =
                -translationController.calculate(pose.getTranslation().getDistance(targetPose.getTranslation()));

        double rotationOutput = rotationController.calculate(pose.getRotation().getRadians());
        drive.driveFieldCentric(
                MetersPerSecond.of(translationOutput * direction.getX()),
                MetersPerSecond.of(translationOutput * direction.getY()),
                RadiansPerSecond.of(rotationOutput));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return translationController.atSetpoint() && rotationController.atSetpoint();
    }
}
