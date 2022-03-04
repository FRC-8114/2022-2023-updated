// Create a low goal and a method to roll out enemy balls. High goal shooter once finished with other two.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
    // Shooter motor controllers
    public final static CANSparkMax shooterController = new CANSparkMax(ShooterConstants.SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    public final static CANSparkMax upperKickerController = new CANSparkMax(ShooterConstants.UPPER_KICKER_CONTROLLER_PORT, MotorType.kBrushed);
    public final static CANSparkMax lowerKickerController = new CANSparkMax(ShooterConstants.LOWER_KICKER_CONTROLLER_PORT, MotorType.kBrushed);

    // Shooter motor controller encoders
    final RelativeEncoder shooterControllerEncoder = shooterController.getEncoder();

    // Shooter variables
	public static double ShooterRPM = 0;
    public static double ShooterVoltage = 0;

    // Creates the ShooterSubsystem
    public ShooterSystem () {
        shooterController.restoreFactoryDefaults();
        shooterController.setIdleMode(IdleMode.kCoast);
        shooterController.setInverted(Constants.ShooterConstants.SHOOTER_INVERSED);

        shooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        shooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        upperKickerController.setIdleMode(IdleMode.kCoast);
        upperKickerController.setInverted(Constants.ShooterConstants.UPPER_KICKER_INVERSED);

        lowerKickerController.setIdleMode(IdleMode.kBrake);
        lowerKickerController.setInverted(Constants.ShooterConstants.LOWER_KICKER_INVERSED);
        
    }

    // Run repeatedly to update certain values
    public void periodic () {
        ShooterRPM = shooterControllerEncoder.getVelocity();
        ShooterVoltage = shooterController.getBusVoltage();
        SmartDashboard.putNumber("rpm", ShooterRPM);
        SmartDashboard.putNumber("voltage", ShooterVoltage);
    }

    // Ensure that the speed passed into a function is not over the maximum input
    public double verifyVelocity (double speed) {
        int sign = (int) (speed / Math.abs(speed));
        if (Math.abs(speed) > ShooterConstants.MAX_INPUT)
            return sign * ShooterConstants.MAX_INPUT;
        return speed;
    }

    // Run the shooter at a speed from 0 to 1.0
    public void ShooterRun (double speed) {
        shooterController.set(speed);
    }

    // Run the shooter at a given voltage
    public void ShooterRunVoltage (double voltage) {
        shooterController.setVoltage(voltage);
    }

    // Return the current voltage fed into the shooter
    public void getShooterVoltage () {
        shooterController.getBusVoltage();
    }

    // Stop any motion of the shooter
    public void ShooterStop () {
        shooterController.setVoltage(0);
    }

    // Reverse the shooter at a speed from 0 to 1.0
    public void ShooterReverse (double speed) {
        shooterController.set(-speed);
    }

    // Reverse the shooter at a given voltage
    public void ShooterReverseVoltage (double voltage) {
        shooterController.setVoltage(-voltage);
    }

    // Run the upper kicker at a speed from 0 to 1.0
    public void UpperKickerRun (double speed) {
        upperKickerController.set(speed);
    }
    
    // Run the upper kicker at a given voltage
    public void UpperKickerRunVoltage (double voltage) {
        upperKickerController.setVoltage(voltage);
    }

    // Reverse the upper kicker at a speed from 0 to 1.0
    public void UpperKickerReverse (double speed) {
        upperKickerController.set(-speed);
    }

    // Reverse the upper kicker at a given voltage
    public void UpperKickerReverseVoltage (double voltage) {
        upperKickerController.setVoltage(-voltage);
    }

    // Stop any motion of the upper kicker
    public void UpperKickerStop () {
        upperKickerController.setVoltage(0);
    }
    
    // Run the lower kicker at a speed from 0 to 1.0
    public void LowerKickerRun (double speed) {
        lowerKickerController.set(speed);
    }

    // Run the lower kicker at a given voltage
    public void LowerKickerRunVoltage (double voltage) {
        lowerKickerController.setVoltage(voltage);
    }

    // Reverse the lower kicker at a speed from 0 to 1.0
    public void LowerKickerReverse (double speed) {
        lowerKickerController.set(-speed);
    }

    // Reverse the lower kicker at a given voltage
    public void LowerKickerReverseVoltage (double voltage) {
        lowerKickerController.setVoltage(-voltage);
    }

    // Stop any motion of the lower kicker
    public void LowerKickerStop () {
        lowerKickerController.setVoltage(0);
    }

}