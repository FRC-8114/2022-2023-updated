package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class AutoShoot extends CommandBase {
    private int desired_rpm = 0, count = 0;

    public AutoShoot(int desired_rpm) {
        this.desired_rpm = desired_rpm;
    }

    public void initialize() {
        ShooterSystem.ShooterRun(desired_rpm);
    }

    public void execute() {
        if (!(ShooterSystem.ShooterRPM < desired_rpm*.95)) {
            ShooterSystem.UpperKickerRun(1);
            ShooterSystem.LowerKickerRun(1);
            count++;
        }
        else {
            ShooterSystem.UpperKickerStop();
            ShooterSystem.LowerKickerStop();
        }
    }

    public void end() {
        ShooterSystem.ShooterStop();
        ShooterSystem.UpperKickerStop();
        ShooterSystem.LowerKickerStop();
    }

    public boolean isFinished() {
        if (count >= 1)
            return true;
        return false;
    }
}
