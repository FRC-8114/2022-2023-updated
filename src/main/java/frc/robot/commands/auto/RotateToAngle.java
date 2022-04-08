package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotUtils;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class RotateToAngle extends CommandBase {
    private DriveSystem driveSystem;
    private FieldPositioningSystem fieldPositioningSystem;

    private double desiredAngle, velocity, marginOfError, angleDifference, startingError;
    private final double rampRate = 1;

    public RotateToAngle(DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, double desiredAngle, double velocity) {
        this.driveSystem = driveSystem;
        this.fieldPositioningSystem = fieldPositioningSystem;

        this.desiredAngle = desiredAngle;
        this.velocity = velocity;
        marginOfError = 2;
    }

    public void initialize() {
        fieldPositioningSystem.zeroPosition();
        marginOfError = 2;

        startingError = Math.abs(desiredAngle);
    
        SmartDashboard.putNumber("desiredAngle", desiredAngle);

        driveSystem.setRampRate(rampRate);
    }

    public void execute() {
        double angle = (fieldPositioningSystem.navxAngle < 0 ? fieldPositioningSystem.navxAngle + 360 : fieldPositioningSystem.navxAngle);

        angleDifference = Math.abs(desiredAngle - angle); // Current angle discrepancy
        double proportion = angleDifference / startingError;

        // Determine the drive power to use
        double proportionalPower = velocity * proportion;
        double minPower = .35;
        double power = Math.max(proportionalPower, minPower);


        if (fieldPositioningSystem.navxAngle < 0)
            driveSystem.tankDrive(-power, power);
        else
            driveSystem.tankDrive(power, -power);

        RobotUtils.sendToShuffleboard("angleDifference", angleDifference);
    }

    public void end(boolean interrupted) {
        driveSystem.tankDrive(0,0);
    }

    public boolean isFinished() {
        return angleDifference <= marginOfError;
    }
}