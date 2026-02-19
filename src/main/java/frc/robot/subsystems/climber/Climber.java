package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final Hook front;
    private final Hook back;

    public Climber(HookIO frontIO, HookIO backIO) {
        this.front = new Hook(frontIO, "Front");
        this.back = new Hook(backIO, "Back");

        SmartDashboard.putData("Climb", climb());
        SmartDashboard.putData("Stow climb", stow());
        SmartDashboard.putData("Extend climb", extend());
        SmartDashboard.putData("Zero climb", zero());
    }

    @Override
    public void periodic() {}

    public Command climb() {
        return front.climb().alongWith(back.climb()).withName("Climb");
    }

    public Command stow() {
        return front.stow().alongWith(back.stow()).withName("Stow climb");
    }

    public Command extend() {
        return front.extend().alongWith(back.extend()).withName("Extend climb");
    }

    public Command zero() {
        return front.zero().alongWith(back.zero()).withName("Zero climb");
    }
}
