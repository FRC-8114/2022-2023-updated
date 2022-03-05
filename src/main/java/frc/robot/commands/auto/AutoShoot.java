package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSystem;

public class AutoShoot extends CommandBase {
    private ShooterSystem shooterSystem;
    private int desired_rpm = 0, count = 0;
    private Timer timer;
    private double shootingTime;

    public AutoShoot(int desired_rpm, ShooterSystem shooterSystem) {
        this.desired_rpm = desired_rpm;
        this.shooterSystem = shooterSystem;

        timer = new Timer();
    }

    public void initialize() {
        shooterSystem.ShooterRunVoltage((desired_rpm - Constants.RPM_TO_VOLTAGE_CONSTANT)/ Constants.RPM_TO_VOLTAGE);

        timer.start();
    }

    public void execute() {
        if (ShooterSystem.ShooterRPM >= desired_rpm) {
            shooterSystem.UpperKickerRun(1);
            shooterSystem.LowerKickerRun(1);
            if(shootingTime == 0) {
                shootingTime = timer.get();
            }
            if(timer.get() > shootingTime + 1) {
                count++;
            }
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
