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
        marginOfError = 0.5;
        if(velocity > 0.2) {
            marginOfError *= 2;
        }
        
        maxDriveInput = NetworkTableInstance.getDefault().getTable("Control Variables").getEntry("maxDriveInput");
        formerMaxDriveInput = maxDriveInput.getDouble(Constants.DriveConstants.INITIAL_MAX_INPUT);
        maxDriveInput.forceSetDouble(0.05);
        driveSystem.setMaxInput(0.05);
    }

    public void execute() {
        double angleDifference = desiredAngle - fieldPositioningSystem.angle;

        if((angleDifference < 0 && Math.abs(angleDifference) < 180) || (angleDifference > 0 && Math.abs(angleDifference) > 180)) {
            driveSystem.tankDrive(-0.8, 0.8);
        } else {
            driveSystem.tankDrive(0.8, -0.8);
        }
    }

    public void end() {
        driveSystem.arcadeDrive(0,0);
        maxDriveInput.forceSetDouble(formerMaxDriveInput);
        driveSystem.setMaxInput(formerMaxDriveInput);
    }

    public boolean isFinished() {
        return Math.abs(desiredAngle - fieldPositioningSystem.angle) <= marginOfError;
    }
}