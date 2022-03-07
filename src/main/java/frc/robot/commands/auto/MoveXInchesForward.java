package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotUtils;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class MoveXInchesForward extends CommandBase {
    private double[] startingPos;
    private double desiredDistance, velocity;
    private DriveSystem driveSystem;
    private FieldPositioningSystem fieldPositioningSystem;

    public MoveXInchesForward(DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, double desiredDistance, double velocity) {
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

        // Move at the given velocity
        driveSystem.tankDrive(-velocity, -velocity);

        // Sent for debugging purposes
        RobotUtils.sendNumberToShuffleboard("autoDistance", fieldPositioningSystem.averageEncoderDistance());
        RobotUtils.sendNumberToShuffleboard("traveledDistance", fieldPositioningSystem.distanceFrom(startingPos));
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
     * @return determines wether the command terminates
     */
    public boolean isFinished() {
        // Terminate if you have traveled the desired distance with a margin of error of 0.05 inches
        return desiredDistance <= fieldPositioningSystem.distanceFrom(startingPos) - 0.05;
    }
}
