package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PositioningConstants;
import frc.robot.subsystems.DriveSystem;

public class Rotate2 extends CommandBase{
    DriveSystem driveSystem;
    private double degrees, velocity;
    //private final double radius = PositioningConstants.DISTANCE_BETWEEN_WHEELS / 2;
    private final double radius = 3.32;

    private int direction;
    private double marginOfErrorDegrees = .25;
    private double marginOfErrorDistance;
    private double oldLeftEncoderPosition, changeInLeftEncoderPosition, distanceToMove;
    public Rotate2 (DriveSystem driveSystem, double degrees, double velocity) {
        this.driveSystem = driveSystem;
        this.degrees = degrees;
        this.velocity = velocity;

        changeInLeftEncoderPosition = 0;

    }
    public void initialize() {
        oldLeftEncoderPosition = -1 * driveSystem.getLeftDistance();
        if (velocity > .5)
            marginOfErrorDegrees *= 2;

        if (degrees > 0)
            direction = 1;
        else
            direction = -1;

        distanceToMove = 0.5 * Math.toRadians(degrees) * Math.pow(radius, 2);
        marginOfErrorDistance = 0.5 * Math.toRadians(marginOfErrorDegrees) * Math.pow(radius, 2);

    }
    public void execute() {
        driveSystem.tankDrive(direction * -velocity, direction * velocity);
        changeInLeftEncoderPosition += -1 * driveSystem.getLeftDistance() - oldLeftEncoderPosition;
        
        oldLeftEncoderPosition = -1 * driveSystem.getLeftDistance();
        
        SmartDashboard.putNumber("Rotate2 distanceToMove", distanceToMove);
        SmartDashboard.putNumber("Rotate2 changeInLeftEncoderPosition", changeInLeftEncoderPosition);

    }
    public void end(boolean interrupted) {
        driveSystem.tankDrive(0, 0);

    }
    public boolean isFinished() {
        if (changeInLeftEncoderPosition > distanceToMove - marginOfErrorDistance &&
            changeInLeftEncoderPosition < distanceToMove + marginOfErrorDegrees)
            return true;
        return false;

    }
    
}
