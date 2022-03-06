package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class Rotate extends CommandBase {
    FieldPositioningSystem positioningSystem;
    DriveSystem driveSystem;
    private double currentAngle, endAngle, velocity, marginOfError;
    private int direction;
    public Rotate (DriveSystem driveSystem, FieldPositioningSystem positioningSystem, double degrees, double velocity){
        this.driveSystem = driveSystem;
        this.positioningSystem = positioningSystem;

        currentAngle = positioningSystem.angle;
        if (currentAngle < 0)
            currentAngle = 2 * Math.PI + currentAngle;
        degrees = Math.toRadians(degrees);
        endAngle = currentAngle + degrees;
        if (endAngle >= 2 * Math.PI)
            endAngle -= 2 * Math.PI;
        this.velocity = velocity;
        marginOfError = .1;

    }
    public void initialize() {


    }
    public void execute() {
        currentAngle = positioningSystem.angle;
        if (currentAngle < 0)
            currentAngle = 2 * Math.PI + currentAngle;
        driveSystem.tankDrive(direction * -velocity, direction * velocity);

    }
    public void end() {
        driveSystem.tankDrive(0, 0);

    }
    public boolean isFinished() {
        if (endAngle - currentAngle <= marginOfError)
            return true;
        return false;

    }

}
