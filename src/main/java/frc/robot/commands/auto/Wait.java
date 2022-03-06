package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double seconds;

    private Timer timer;
    public Wait (double seconds) {
        this.seconds = seconds; 

    }
    public void initialize() {
        timer.start();

    }
    public void execute() {
        
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
