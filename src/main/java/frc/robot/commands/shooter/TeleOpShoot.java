package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;

public class TeleOpShoot extends CommandBase {
    private double lowerKickerSpeed, upperKickerSpeed;
    private ShooterSystem shooterSystem;

    public TeleOpShoot(double lowerKickerSpeed, double upperKickerSpeed, ShooterSystem shooterSystem) {
        this.lowerKickerSpeed = lowerKickerSpeed;
        this.upperKickerSpeed = upperKickerSpeed;
        this.shooterSystem = shooterSystem;

    }
    public void initialize() {

    }
    public void execute() {
        shooterSystem.ShooterRunVoltage(ShooterConstants.TELEOP_DESIRED_RPM*Constants.RPM_TO_VOLTAGE + Constants.RPM_TO_VOLTAGE_CONSTANT);
        if (shooterSystem.ShooterRPM > ShooterConstants.TELEOP_DESIRED_RPM*.975) {
            shooterSystem.LowerKickerRun(lowerKickerSpeed);
            shooterSystem.UpperKickerRun(upperKickerSpeed);

        }
            
    }

    public void end() {
    }

    public boolean isFinished() {
        return true;

    }
    
}
