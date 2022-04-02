package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class RotateToAngle extends CommandBase {
    private DriveSystem driveSystem;
    private FieldPositioningSystem fieldPositioningSystem;

    private double desiredAngle, velocity, marginOfError, angleDifference;
    private boolean clockwise;

    public RotateToAngle(DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, double desiredAngle, double velocity) {
        this.driveSystem = driveSystem;
        this.fieldPositioningSystem = fieldPositioningSystem;

        this.desiredAngle = desiredAngle;
        this.velocity = velocity;
        clockwise = true;
    }

    public void initialize() {
        marginOfError = 0.5;
        if(velocity > 0.5) {
            marginOfError *= 2;
        }

        if(fieldPositioningSystem.angle + 180 > desiredAngle) {
            clockwise = false;
        }
    
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
    }

    public void execute() {
        angleDifference = Math.abs(desiredAngle - fieldPositioningSystem.angle);

        if(angleDifference <= 4) {
            if(clockwise) {
                driveSystem.tankDrive(-velocity * .85, velocity * .85);
            } else {
                driveSystem.tankDrive(velocity * .85, -velocity * .85);
            }
        } else {
            if(clockwise) {
                driveSystem.tankDrive(-velocity, velocity);
            } else {
                driveSystem.tankDrive(velocity, -velocity);
            }
        }

        SmartDashboard.putNumber("angleDifference", angleDifference);
    }

    public void end(boolean interrupted) {
        driveSystem.arcadeDrive(0,0);
    }

    public boolean isFinished() {
        return angleDifference <= marginOfError;
    }
}