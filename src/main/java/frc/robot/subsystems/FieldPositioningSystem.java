package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotUtils;

public class FieldPositioningSystem extends SubsystemBase {
    public AHRS navx;
    public double navxAngle, angleOffset;
    public double angleUsingEncoders;
    public double[] locationUsingNavx;
    public double[] locationUsingEncoders;
   
    private DriveSystem driveSystem;
    private double radius = Constants.PositioningConstants.DISTANCE_BETWEEN_WHEELS / 2;
    private double[] oldLeftEncoderValues = new double[2], oldRightEncoderValues = new double[2];

    public FieldPositioningSystem(DriveSystem driveSystem) {
        this.driveSystem = driveSystem;
        initializePosition(new double[] {0, 0}, 0);

    }

    public FieldPositioningSystem(DriveSystem driveSystem, double[] position, double angle) {
        this.driveSystem = driveSystem;
        initializePosition(position, angle);

    }

    /**
     * Converts motor shaft rotations into distance, accounting for both the
     * gearbox and wheel size
     * 
     * @param rotations rotations of the motor
     */
    public double rotationsToDistance(double rotations) {
        return rotations * Constants.PositioningConstants.GEAR_RATIO * Constants.PositioningConstants.WHEEL_CIRCUMFRENCE;
    }

    /**
     * Sets the robot's location to 0,0 and the angle to zero
     */
    public void zeroPosition() {
        overwritePosition(new double[] {0,0}, 0);
    }

    /**
     * Sets the robot's position to match the given location and angle
     *
     * @param location
     * @param angle
     */
    public void overwritePosition(double[] location, double angle) {
        overwriteLocation(location);
        overwriteAngle(angle);

    }

    /**
     * Sets the location of the robot to be the given location
     * 
     * @param location
     */
    public void overwriteLocation(double[] location) {
        locationUsingNavx = locationUsingEncoders = location;
    }

    /**
     * Sets the angle of the robot to be the given angle
     * 
     * @param angle
     */
    public void overwriteAngle(double angle) {
        //navxAngle
        navx.zeroYaw();
        angleOffset = angle;
        //angleUsingEncoders
        angleUsingEncoders = angle;

    }

    /**
     * Updates the values for navx angle and location using navx by
     * finding the distance covered and converting it to x displacement
     * and y displacement
     */
    public void updatePositionUsingNavx() {
        //navxAngle
        navxAngle = navx.getYaw() + angleOffset;
        //locationUsingNavx
        double distanceCovered = rotationsToDistance(averageEncoderDistance());
        updateEncoderValues();
        double xDisplacement = distanceCovered * Math.cos(Math.toRadians(navxAngle));
        double yDisplacement = distanceCovered * Math.sin(Math.toRadians(navxAngle));
        locationUsingNavx[0] += xDisplacement;
        locationUsingNavx[1] += yDisplacement;

    }

    /**
     * Updates the angle using encoders or the location using encoders
     * depending on whether the robot is spinning or driving straight
     */
    public void updatePositionUsingEncoders() {
        //angleUsingEncoders
        double leftDisplacement = rotationsToDistance(averageLeftDistance());
        double rightDisplacement = rotationsToDistance(averageRightDistance());
        //if (Math.signum(-leftDisplacement) == Math.signum(rightDisplacement))
            angleUsingEncoders += Math.toDegrees(leftDisplacement / radius);
        RobotUtils.sendToShuffleboard("left displacement divided by radius", leftDisplacement);
        //locationUsingEncoders
        /*
        else {
            double distanceCovered = rotationsToDistance(averageEncoderDistance());
            double xDisplacement = distanceCovered * Math.cos(Math.toRadians(angleUsingEncoders));
            double yDisplacement = distanceCovered * Math.sin(Math.toRadians(angleUsingEncoders));
            locationUsingEncoders[0] += xDisplacement;
            locationUsingEncoders[1] += yDisplacement;

        }
        */
        updateEncoderValues();

    }

    /**
     * Updates the old encoder value variables to have the current values
     */
    public void updateEncoderValues() {
        oldLeftEncoderValues[0] = driveSystem.leftLeaderEncoder.getPosition();
        oldLeftEncoderValues[1] = driveSystem.leftFollowerEncoder.getPosition();
        oldRightEncoderValues[0] = driveSystem.rightLeaderEncoder.getPosition();
        oldRightEncoderValues[1] = driveSystem.rightFollowerEncoder.getPosition();

    }

    public double averageLeftDistance() {
        double averageOldLeftEncoders = (oldLeftEncoderValues[0] + oldLeftEncoderValues[1]) / 2;
        return driveSystem.getLeftDistance() - averageOldLeftEncoders;

    }

    public double averageRightDistance() {
        double averageOldRightEncoders = (oldRightEncoderValues[0] + oldRightEncoderValues[1]) / 2;
        return driveSystem.getRightDistance() - averageOldRightEncoders;

    }

    /**
     * Returns the average displacement of the encoders since the last update
     */
    public double averageEncoderDistance() {
        double averageOldEncoders = (oldLeftEncoderValues[0] + oldLeftEncoderValues[1] + oldRightEncoderValues[0] + oldRightEncoderValues[1]) / 4;
        return driveSystem.getTotalDistance() - averageOldEncoders;

    }

    /**
     * Provides the distance from the current position to the provided
     * distancePos using variables affected by the navx
     * 
     * @param distancePos
     * @return distanceFromTo method
     */
    public double distanceToPointNavx(double[] distancePos) {
        return distanceFromTo(locationUsingNavx, distancePos);
    }

    /**
     * Provides the distance from the current position to the provided
     * distancePos using variables affected by the navx
     * 
     * @param distancePos
     * @return distanceFromTo method
     */
    public double xDistanceToPointNavx(double[] distancePos) {
        return Math.abs(locationUsingNavx[0] - distancePos[0]);
    }

    /**
     * Provides the distance from the current position to the provided
     * distancePos using variables affected by the navx
     * 
     * @param distancePos
     * @return distanceFromTo method
     */
    public double yDistanceToPointNavx(double[] distancePos) {
        return Math.abs(locationUsingNavx[1] - distancePos[1]);
    }

    /**
     * Provides the distance from the current position to the provided
     * distancePos using variables affected by encoder values
     * 
     * @param distancePos
     * @return distanceFromTo method
     */
    public double distanceToPointEncoders(double[] distancePos) {
        return distanceFromTo(locationUsingEncoders, distancePos);
    }

    /**
     * Outputs the distance between two sets of coordinates using pythagoras's
     * theorem
     * 
     * @param pos1
     * @param pos2
     * @return distance
     */
    public double distanceFromTo(double[] pos1, double[] pos2) {
        double xDistance = pos2[0] - pos1[0];
        double yDistance = pos2[1] - pos1[1];
        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    }

    /**
     * Provides the angle from the current position to the provided
     * anglePos using variables affected by the navx
     * 
     * @param anglePos
     * @return angleFromTo method
     */
    public double angleToPointNavx(double[] anglePos) {
        return angleFromTo(locationUsingNavx, anglePos);
    }

    /**
     * Provides the angle from the current position to the provided
     * anglePos using variables affected by encoded values
     * 
     * @param anglePos
     * @return angleFromTo method
     */
    public double angleToPointEncoders(double[] anglePos) {
        return angleFromTo(locationUsingEncoders, anglePos);
    }

    /**
     * Outputs the angle between two sets of coordinates using arc
     * tangent calculation
     * @param pos1
     * @param pos2
     * @return angle
     */
    public double angleFromTo(double[] pos1, double[] pos2) {
        double xDistance = pos2[0] - pos1[0];
        double yDistance = pos2[1] - pos1[1];

        if (xDistance != 0)
            return Math.toDegrees(Math.atan(yDistance / xDistance));
        else if (yDistance > 0)
            return 90;
        return -90;

    }

    /**
     * Initializes the position of the robot when this class object is created
     * 
     * @param location
     * @param angle
     */
    private void initializePosition(double[] location, double angle) {
            //location
            locationUsingNavx = locationUsingEncoders = location;
            //angle
            navx = new AHRS(SerialPort.Port.kUSB);
            navx.calibrate();
            this.angleOffset = angle;
            navxAngle = angleUsingEncoders = navx.getYaw() + angleOffset;

    }

}