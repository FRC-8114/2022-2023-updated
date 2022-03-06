package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;

public class AutoShoot extends CommandBase {
    private ShooterSystem shooterSystem;
    private int count = 0;
    private Timer timer;

    public AutoShoot(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;

    }

    public void initialize() {
        timer.start();
    }

    public void execute() {
        shooterSystem.ShooterRunVoltage(ShooterConstants.AUTO_DESIRED_VOLTAGE);

        if (timer.get() > ShooterConstants.SHOOTER_SPIN_UP_TIME) {
            shooterSystem.UpperKickerRun(1);
            shooterSystem.LowerKickerRun(1);
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
        SmartDashboard.putNumber("voltage when stopped!!", shooterSystem.getShooterVoltage());
    }

    public boolean isFinished() {
        if (timer.get() > 4)
            return true;
        return false;
    }
}
