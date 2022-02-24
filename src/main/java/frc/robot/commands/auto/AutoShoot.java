package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class AutoShoot extends CommandBase {
    private ShooterSystem shooterSystem;
    private int desired_rpm = 0, count = 0;

    public AutoShoot(int desired_rpm, ShooterSystem shooterSystem) {
        this.desired_rpm = desired_rpm;
        this.shooterSystem = shooterSystem;
    }

    public void initialize() {
        shooterSystem.ShooterRun(desired_rpm);
    }

    public void execute() {
        if (ShooterSystem.ShooterRPM >= desired_rpm*.95) {
            shooterSystem.UpperKickerRun(1);
            shooterSystem.LowerKickerRun(1);
            count++;

        }
        else {
            shooterSystem.UpperKickerStop();
            shooterSystem.LowerKickerStop();
            
        }
    }

    public void end() {
        shooterSystem.ShooterStop();
        shooterSystem.UpperKickerStop();
        shooterSystem.LowerKickerStop();
    }

    public boolean isFinished() {
        if (count >= 1)
            return true;
        return false;
    }
}
