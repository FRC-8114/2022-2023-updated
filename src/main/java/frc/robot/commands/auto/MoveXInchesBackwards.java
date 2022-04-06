package frc.robot.commands.auto;

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
        oldLeftPosition = fieldPositioningSystem.rotationsToDistance(driveSystem.getLeftDistance());
        oldRightPosition = fieldPositioningSystem.rotationsToDistance(driveSystem.getRightDistance());

        RobotUtils.sendToShuffleboard("desiredDistance", desiredDistance);
    }

    /**
     * This method runs every robot tick between the command being scheduled
     * and the command terminating
     */
    public void execute() {
        changeInLeftPosition += fieldPositioningSystem.rotationsToDistance(driveSystem.getLeftDistance()) - oldLeftPosition;
        changeInRightPosition += fieldPositioningSystem.rotationsToDistance(driveSystem.getRightDistance()) - oldRightPosition;
        // Move at the given velocity if we are less than 1 tenth an inch off straight
        if(Math.abs(changeInLeftPosition - changeInRightPosition) <= 0.1) {
            driveSystem.tankDrive(velocity, velocity);
        }
        else if(Math.abs(changeInLeftPosition) > Math.abs(changeInRightPosition)) {
            driveSystem.tankDrive(velocity * .98, velocity);
        } 
        else {
            driveSystem.tankDrive(velocity, velocity * .98);
        }
        oldLeftPosition = fieldPositioningSystem.rotationsToDistance(driveSystem.getLeftDistance());
        oldRightPosition = fieldPositioningSystem.rotationsToDistance(driveSystem.getRightDistance());
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
        return fieldPositioningSystem.distanceFrom(startingPos) >= desiredDistance - 0.05;
    }
}
