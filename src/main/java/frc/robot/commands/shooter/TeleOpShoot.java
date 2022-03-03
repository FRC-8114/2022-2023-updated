package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSystem;

public class TeleOpShoot extends CommandBase {
    private double desiredRPM, lowerKickerSpeed, upperKickerSpeed;
    private ShooterSystem shooterSystem;

    public TeleOpShoot(double desiredRPM, double lowerKickerSpeed, double upperKickerSpeed, ShooterSystem shooterSystem) {
        this.desiredRPM = desiredRPM;
        this.lowerKickerSpeed = lowerKickerSpeed;
        this.upperKickerSpeed = upperKickerSpeed;
        this.shooterSystem = shooterSystem;

    }
    public void initialize() {

    }
    public void execute() {
        shooterSystem.ShooterRunVoltage(desiredRPM * Constants.RPM_TO_VOLTAGE + Constants.RPM_TO_VOLTAGE_CONSTANT);
        if (shooterSystem.ShooterRPM >= .95 * desiredRPM) {
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
