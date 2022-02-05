package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldPositioningSystem extends SubsystemBase {
    private DriveSystem drive;
    public double[] position;
    public double[] oldLeftEncoderValues = new double[2], oldRightEncoderValues = new double[2];
    public double angle;

    public FieldPositioningSystem(DriveSystem drive) {
        this.drive = drive;
        double[] position = new double[2];
        position[0] = 0;
        position[1] = 0;
        
        initializePosition(position, 0);
    }

    public FieldPositioningSystem(DriveSystem drive, double[] position, double angle) {
        this.drive = drive;
        
        initializePosition(position, angle);
    }

    public void initializePosition(double[] position, double angle) {
        this.position = position;
        this.angle = angle;
    }

    /**
     * Updates the values for the current displacement vector before applying
     * it to the robot's current position
     */
    public void updatePosition() {
        double distanceCovered = averageEncoderDistance();
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
}