package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class Rotate extends CommandBase {
    FieldPositioningSystem positioningSystem;
    DriveSystem driveSystem;
    private double currentAngle, endAngle, marginOfError, velocity;
    private int direction;
    public Rotate (DriveSystem driveSystem, FieldPositioningSystem positioningSystem, double degrees, double marginOfError, double velocity){
        this.driveSystem = driveSystem;
        this.positioningSystem = positioningSystem;

        currentAngle = positioningSystem.angle;
        //convert current angle to unit circle
        if (currentAngle < 0)
            currentAngle = 2 * Math.PI + currentAngle;
        degrees = Math.toRadians(degrees);
        endAngle = currentAngle + degrees;
        this.marginOfError = Math.PI * marginOfError / 180;
        this.velocity = velocity;

    }
    public void initialize() {


    }
    public void execute() {
        currentAngle = positioningSystem.angle;
        //convert current angle to unit circle
        if (currentAngle < 0)
            currentAngle = 2 * Math.PI + currentAngle;
        driveSystem.tankDrive(direction * -velocity, direction * velocity);

    }
    public void end(boolean interrupted) {
        driveSystem.tankDrive(0, 0);

    }
    public boolean isFinished() {
        if (Math.abs(endAngle - currentAngle) <= marginOfError || Math.abs(endAngle - (2 * Math.PI + currentAngle)) <= marginOfError)
            return true;
        return false;

    }

}
