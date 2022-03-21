package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotUtils;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class MoveXInchesBackwards extends CommandBase {
    private double[] startingPos;
    private double desiredDistance, velocity, changeInLeftPosition, changeInRightPosition, oldLeftPosition, oldRightPosition;
    private DriveSystem driveSystem;
    private FieldPositioningSystem fieldPositioningSystem;

    public MoveXInchesBackwards(DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, double desiredDistance, double velocity) {
        this.driveSystem = driveSystem;
        this.fieldPositioningSystem = fieldPositioningSystem;
        
        this.desiredDistance = desiredDistance;
        this.velocity = velocity;
    }

    /**
     * This method runs whenever the command is scheduled
     */
    public void initialize() {
        startingPos = new double[2];
        startingPos[0] = fieldPositioningSystem.position[0];
        startingPos[1] = fieldPositioningSystem.position[1];

        RobotUtils.sendNumberToShuffleboard("desiredDistance", desiredDistance);
    }

    /**
     * This method runs every robot tick between the command being scheduled
     * and the command terminating
     */
    public void execute() {
        changeInLeftPosition += driveSystem.getLeftDistance() - oldLeftPosition;
        changeInRightPosition += driveSystem.getRightDistance() - oldRightPosition;
        oldLeftPosition = driveSystem.getLeftDistance();
        oldRightPosition = driveSystem.getRightDistance();
        // Move at the given velocity if we are less than 1 tenth an inch off straight
        if(Math.abs(Math.abs(changeInLeftPosition) - Math.abs(changeInRightPosition)) <= 0.1) {
            driveSystem.tankDrive(velocity, velocity);
        } else if(Math.abs(changeInLeftPosition) > Math.abs(changeInRightPosition)) {
            driveSystem.tankDrive(velocity * .98, velocity);
        } else {
            driveSystem.tankDrive(velocity, velocity * .98);
        }

        // Sent for debugging purposes
        RobotUtils.sendNumberToShuffleboard("autoDistance", fieldPositioningSystem.averageEncoderDistance());
        RobotUtils.sendNumberToShuffleboard("MoveXInchesForward traveledDistance", fieldPositioningSystem.distanceFrom(startingPos));
        RobotUtils.sendNumberToShuffleboard("MoveXInchesForward traveledDistance from encoders", changeInLeftPosition);
    }

    /**
     * This method runs when the command terminates
     */
    public void end(boolean interrupted) {
        driveSystem.tankDrive(0, 0);
    }

    /**
     * This command determines if the command terminates
     * 
     * @return determines whether the command terminates
     */
    public boolean isFinished() {
        // Terminate if you have traveled the desired distance with a margin of error of 0.05 inches
        return desiredDistance <= fieldPositioningSystem.distanceFrom(startingPos) - 0.05;
    }
}
