package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotUtils;

public class FieldPositioningSystem extends SubsystemBase {
    private DriveSystem drive;
    public AHRS navx;
    public double[] position;
    public double[] oldLeftEncoderValues = new double[2], oldRightEncoderValues = new double[2];
    public double angle, angleOffset;

    public FieldPositioningSystem(DriveSystem drive) {
        this.drive = drive;
        double[] position = {0, 0};
        
        initializePosition(position, 0);
    }

    public FieldPositioningSystem(DriveSystem drive, double[] position, double angle) {
        this.drive = drive;
        
        initializePosition(position, angle);
    }

    public void initializePosition(double[] position, double angle) {
        navx = new AHRS(SerialPort.Port.kUSB);
        navx.calibrate();
        this.position = position;
        this.angleOffset = angle;
        this.angle = navx.getYaw() + angleOffset;
    }

    /**
     * Sets the robot's location to 0,0 and the angle to zero
     */
    public void zeroPosition() {
        overwriteLocation(new double[] {0,0});
        overwriteAngle(0);
    }

    /**
     * Sets the robot's location to match the given point and angle
     */
    public void overwritePosition(double[] point, double angle) {
        overwriteLocation(new double[] {0,0});
        overwriteAngle(angle);
    }

    /**
     * Sets the position of the robot to be the given position
     * 
     * @param position
     */
    public void overwriteLocation(double[] position) {
        this.position = position;
    }

    /**
     * Zeros the yaw angle of the navx
     * 
     * @param position
     */
    public void overwriteAngle(double angle) {
        navx.zeroYaw();

        angleOffset = angle;
    }

    /**
     * Updates the values for the current displacement vector before applying
     * it to the robot's current position
     */
    public void updatePosition() {
        double distanceCovered = averageEncoderDistance();

        angle = navx.getYaw() + angleOffset;
        RobotUtils.sendNumberToShuffleboard("yawDegrees", angle);

        double xDisplacement = distanceCovered * Math.cos(Math.toRadians(angle));
        double yDisplacement = distanceCovered * Math.sin(Math.toRadians(angle));

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
        double leftLeaderDistance = drive.leftLeaderEncoder.getPosition() - oldLeftEncoderValues[0];
        double leftFollowerDistance = drive.leftFollowerEncoder.getPosition() - oldLeftEncoderValues[1];
        double rightLeaderDistance = drive.rightLeaderEncoder.getPosition() - oldRightEncoderValues[0];
        double rightFollowerDistance = drive.rightFollowerEncoder.getPosition() - oldRightEncoderValues[1];

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

    /**
     * Returns the distance along the x-axis from a given position 
     * to the robot
     * 
     * @param pos   the position to check distance from
     * @return      the x distance from the given position
     */
    public double xDistanceFrom(double[] distancePos) {
        return position[0] - distancePos[0];
    }

    /**
     * Returns the distance along the y-axis from a given position 
     * to the robot
     * 
     * @param pos   the position to check distance from
     * @return      the y distance from the given position
     */
    public double yDistanceFrom(double[] distancePos) {
        return position[1] - distancePos[1];
    }

    public double angleToPoint(double[] anglePos) {
        SmartDashboard.putNumber("tan", yDistanceFrom(anglePos) / xDistanceFrom(anglePos));
        SmartDashboard.putNumber("arctan", Math.atan(yDistanceFrom(anglePos) / xDistanceFrom(anglePos)));

        if(xDistanceFrom(anglePos) != 0 && yDistanceFrom(anglePos) != 0) {
            return Math.toDegrees(Math.atan(yDistanceFrom(anglePos) / xDistanceFrom(anglePos)));
        } else if(xDistanceFrom(anglePos) == 0) {
            if(yDistanceFrom(anglePos) > 0) {
                return 90;
            }
            return -90;
        } else {
            if(xDistanceFrom(anglePos) > 0) {
                return 0;
            }
            return 180;
        }
    }
}