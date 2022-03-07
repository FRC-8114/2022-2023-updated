package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class Rotate extends CommandBase {
    FieldPositioningSystem positioningSystem;
    DriveSystem driveSystem;
    private double desiredAngle, marginOfErrorDegrees, velocity;
    public Rotate (DriveSystem driveSystem, FieldPositioningSystem positioningSystem, double degrees, double marginOfErrorDegrees, double velocity){
        this.driveSystem = driveSystem;
        this.positioningSystem = positioningSystem;
        this.desiredAngle = positioningSystem.angle + Math.toRadians(degrees);
        this.marginOfErrorDegrees = marginOfErrorDegrees;

    }
    public void initialize() {
        new RotateToAngle(driveSystem, positioningSystem, desiredAngle, velocity);

    }
    public void execute() {
        

    }
    public void end(boolean interrupted) {
        driveSystem.tankDrive(0, 0);

    }
    public boolean isFinished() {
        return true;

    }

}
