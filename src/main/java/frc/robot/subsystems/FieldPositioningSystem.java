package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotUtils;

public class FieldPositioningSystem extends SubsystemBase {
    private DriveSystem drive;
    public AHRS navx;
    public double[] position;
    public double[] oldLeftEncoderValues = new double[2], oldRightEncoderValues = new double[2];
    public double angle;

    public FieldPositioningSystem(DriveSystem drive) {
        this.drive = drive;
        double[] position = new double[2];
        position[0] = 0;
        position[1] = 0;

        navx = new AHRS(SerialPort.Port.kUSB);
        navx.calibrate();
        
        initializePosition(position, 0);
    }

    public FieldPositioningSystem(DriveSystem drive, double[] position, double angle) {
        this.drive = drive;
        
        initializePosition(position, angle);
    }

    public void initializePosition(double[] position, double angle) {
        this.position = position;
        this.angle = navx.getYaw();
    }

    /**
     * Converts motor shaft rotations into distance, accounts for both the
     * gearbox and wheel size.
     */
    public double rotationsToDistance(double rotations) {
        return rotations * Constants.PositioningConstants.GEAR_RATIO * Constants.PositioningConstants.WHEEL_CIRCUMFRENCE;
    }

    /**
     * Updates the values for the current displacement vector before applying
     * it to the robot's current position
     */
    public void updatePosition() {
        double rotations = averageEncoderDistance();
        double distanceCovered = rotationsToDistance(rotations);

        angle = navx.getYaw();
        RobotUtils.sendNumberToShuffleboard("yawDegrees", angle);

        double xDisplacement = distanceCovered * Math.cos(angle);
        double yDisplacement = distanceCovered * Math.sin(angle);

        position[0] += xDisplacement;
        position[1] += yDisplacement;

        updateEncoderValues();
    }

    /**
     * Updates the old encoder value variables to have the current values
     */
    public void updateEncoderValues() {
        oldLeftEncoderValues[0] = drive.leftLeaderEncoder.getPosition();
        oldLeftEncoderValues[1] = drive.leftFollowerEncoder.getPosition();
        oldRightEncoderValues[0] = drive.rightLeaderEncoder.getPosition();
        oldRightEncoderValues[1] = drive.rightFollowerEncoder.getPosition();
    }

    /**
     * Returns the average displacement of the encoders since the alst update
     */
    public double averageEncoderDistance() {
        double leftLeaderDistance = Math.abs(drive.leftLeaderEncoder.getPosition() - oldLeftEncoderValues[0]);
        double leftFollowerDistance = Math.abs(drive.leftFollowerEncoder.getPosition() - oldLeftEncoderValues[1]);
        double rightLeaderDistance = Math.abs(drive.rightLeaderEncoder.getPosition() - oldRightEncoderValues[0]);
        double rightFollowerDistance = Math.abs(drive.rightFollowerEncoder.getPosition() - oldRightEncoderValues[1]);

        return (leftFollowerDistance + leftLeaderDistance + rightFollowerDistance + rightLeaderDistance) / 4.0;
    }

    /**
     * Returns the distance from a given position to the robot
     * 
     * @param pos   the position to check distance from
     * @return      the distance from the given position
     */
    public double distanceFrom(double[] distancePos) {
        double xDistance = position[0] - distancePos[0];
        double yDistance = position[1] - distancePos[1];
        
        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    }
}