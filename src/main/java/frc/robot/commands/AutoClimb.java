// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants.ClimbPosition;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import java.util.Set;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends SequentialCommandGroup {
    /** Creates a new AutoClimb. */
    public AutoClimb(
            ClimbPosition position, Drive drive, Climber climber, Superstructure superstructure, boolean isAuto) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                superstructure.setGoal(Goal.IDLE),
                climber.extend(),
                new AlignToClimb(position, drive),
                isAuto ? climber.autoClimb() : climber.climb());
    }

    public static Command getAutoClimbCommand(Drive drive, Climber climber, Superstructure superstructure) {
        return Commands.defer(
                () -> {
                    Pose2d pose = drive.getPose();
                    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                        pose = FlippingUtil.flipFieldPose(pose);
                    }
                    boolean front = pose.getTranslation().getX()
                            > (ClimbPosition.FRONT_LEFT.getPose().getX()
                                            + ClimbPosition.BACK_LEFT.getPose().getX())
                                    / 2.0;
                    boolean left = pose.getTranslation().getY()
                            > (ClimbPosition.FRONT_LEFT.getPose().getY()
                                            + ClimbPosition.FRONT_RIGHT
                                                    .getPose()
                                                    .getY())
                                    / 2.0;
                    boolean isAuto = DriverStation.isAutonomous();
                    if (front && left) {
                        return new AutoClimb(ClimbPosition.FRONT_LEFT, drive, climber, superstructure, isAuto);
                    } else if (front) {
                        return new AutoClimb(ClimbPosition.FRONT_RIGHT, drive, climber, superstructure, isAuto);
                    } else if (left) {
                        return new AutoClimb(ClimbPosition.BACK_LEFT, drive, climber, superstructure, isAuto);
                    } else {
                        return new AutoClimb(ClimbPosition.BACK_RIGHT, drive, climber, superstructure, isAuto);
                    }
                },
                Set.of(drive, climber, superstructure));
    }
}
