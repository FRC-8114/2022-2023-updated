package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.auto.Wait;
import frc.robot.subsystems.ShooterSystem;

public class TeleOpShoot extends CommandBase {
    private double lowerKickerSpeed, upperKickerSpeed;
    private ShooterSystem shooterSystem;
    private boolean done;

    public TeleOpShoot(double lowerKickerSpeed, double upperKickerSpeed, ShooterSystem shooterSystem) {
        this.lowerKickerSpeed = lowerKickerSpeed;
        this.upperKickerSpeed = upperKickerSpeed;
        this.shooterSystem = shooterSystem;

    }

    public void initialize() {

    }

    public void execute() {
             
    }

    public boolean isFinished() {
        return done;
    }
    
}
