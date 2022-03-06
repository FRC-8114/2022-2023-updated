package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class Wait extends CommandBase {
    DriveSystem driveSystem;
    private double seconds;

    private Timer timer;
    public Wait (DriveSystem driveSystem, double seconds) {
        this.driveSystem = driveSystem;
        this.seconds = seconds; 

    }
    public void initialize() {
        timer.start();

    }
    public void execute() {
        driveSystem.tankDrive(0, 0);
        
    }
    public void end() {
        timer.reset();

    }
    public boolean isFinished() {
        if (timer.get() >= seconds)
            return true;
        return false;

    }
    
}
