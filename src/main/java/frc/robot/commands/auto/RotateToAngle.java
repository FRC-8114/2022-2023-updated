package frc.robot.commands.auto;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class RotateToAngle extends CommandBase {
    private DriveSystem driveSystem;
    private FieldPositioningSystem fieldPositioningSystem;
    private NetworkTableEntry maxDriveInput;

    private double desiredAngle, velocity, marginOfError, formerMaxDriveInput;

    public RotateToAngle(DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, double desiredAngle, double velocity) {
        this.driveSystem = driveSystem;
        this.fieldPositioningSystem = fieldPositioningSystem;

        this.desiredAngle = desiredAngle;
        this.velocity = velocity;
    }

    public void initialize() {
        marginOfError = 0.25;
        if(velocity > 0.2) {
            marginOfError *= 2;
        }
        
        maxDriveInput = NetworkTableInstance.getDefault().getTable("Control Variables").getEntry("maxDriveInput");
        formerMaxDriveInput = maxDriveInput.getDouble(Constants.DriveConstants.INITIAL_MAX_INPUT);
    
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
    }

    public void execute() {
        double angleDifference = desiredAngle - fieldPositioningSystem.angle;

        driveSystem.tankDrive(-velocity, velocity);

        SmartDashboard.putNumber("fieldPositioningAngle", fieldPositioningSystem.angle);
        SmartDashboard.putNumber("diff", Math.abs(desiredAngle - fieldPositioningSystem.angle));
        SmartDashboard.putBoolean("done", Math.abs(desiredAngle - fieldPositioningSystem.angle) <= marginOfError);
    }

    public void end() {
        driveSystem.arcadeDrive(0,0);
    }

    public boolean isFinished() {
        return Math.abs(Math.abs(desiredAngle) - Math.abs(fieldPositioningSystem.angle)) <= marginOfError;
    }
}