package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class TeleOpShoot extends CommandBase {
    private double desiredRPM, lowerKickerSpeed, upperKickerSpeed;
    private ShooterSystem shooterSystem;
    private XboxController controller;

    public TeleOpShoot(double desiredRPM, double lowerKickerSpeed, double upperKickerSpeed, ShooterSystem shooterSystem, XboxController controller) {
        this.desiredRPM = desiredRPM;
        this.lowerKickerSpeed = lowerKickerSpeed;
        this.upperKickerSpeed = upperKickerSpeed;
        this.shooterSystem = shooterSystem;
        this.controller = controller;

    }
    public void initialize() {

    }
    public void execute() {
        shooterSystem.ShooterRun(desiredRPM * 1.1);
        if (shooterSystem.ShooterRPM >= desiredRPM) {
            shooterSystem.LowerKickerRun(lowerKickerSpeed);
            shooterSystem.UpperKickerRun(upperKickerSpeed);

        }
            

    }

    public void end() {
        shooterSystem.ShooterStop();
        shooterSystem.UpperKickerStop();
        shooterSystem.LowerKickerStop();

    }

    public boolean isFinished() {
        return true;

    }
    
}
