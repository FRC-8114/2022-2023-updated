package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;

public class TeleOpShoot extends CommandBase {
    private double lowerKickerSpeed, upperKickerSpeed;
    private ShooterSystem shooterSystem;
    private Timer timer;

    public TeleOpShoot(double lowerKickerSpeed, double upperKickerSpeed, ShooterSystem shooterSystem) {
        this.lowerKickerSpeed = lowerKickerSpeed;
        this.upperKickerSpeed = upperKickerSpeed;
        this.shooterSystem = shooterSystem;
        timer = new Timer();

    }
    public void initialize() {
        timer.start();

    }
    public void execute() {
        shooterSystem.ShooterRunVoltage(ShooterConstants.TELEOP_DESIRED_VOLTAGE);
        if (timer.get() > ShooterConstants.SHOOTER_SPIN_UP_TIME) {
            shooterSystem.LowerKickerRun(lowerKickerSpeed);
            shooterSystem.UpperKickerRun(upperKickerSpeed);

        }
            
    }

    public void end() {
        timer.stop();
    }

    public boolean isFinished() {
        return true;

    }
    
}
