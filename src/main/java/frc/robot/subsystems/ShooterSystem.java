// Create a low goal and a method to roll out enemey balls. High goal shooter once finished with other two.

package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.security.KeyStore.Entry;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
    // Shooter motor controllers
    public final static CANSparkMax shooterController = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    public final static CANSparkMax upperKickerController = new CANSparkMax(ShooterConstants.KICKER_CONTROLLER_PORT, MotorType.kBrushless);
    public final static CANSparkMax lowerKickerController = new CANSparkMax(ShooterConstants.KICKER_CONTROLLER_PORT, MotorType.kBrushless);

    // Shooter motor controller encoders
    final RelativeEncoder shooterControllerEncoder = shooterController.getEncoder();
    final RelativeEncoder upperKickerControllerEncoder = upperKickerController.getEncoder();
    final RelativeEncoder lowerKickerControllerEncoder = lowerKickerController.getEncoder();
	public static final double ShooterRPM = 0;

    // Creates the ShooterSubsystem
    public ShooterSystem() {
        shooterController.restoreFactoryDefaults();
        shooterController.setIdleMode(IdleMode.kCoast);
        shooterController.setInverted(true);

        shooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        shooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    }

    public void periodic() {

    }

    public static double verifyVelocity(double speed) {
        int sign = (int) (speed / Math.abs(speed));
        if (Math.abs(speed) > ShooterConstants.MAX_INPUT)
            return sign * ShooterConstants.MAX_INPUT;
        return speed;
    }

    public static void ShooterRun(double speed) {
        shooterController.set(speed);
    }

    public static void ShooterStop() {
        shooterController.stopMotor();
    }

    public void ShooterReverse(double speed) {
        shooterController.set(-speed);
    }

    public static void UpperKickerRun(double speed) {
        upperKickerController.set(speed);
    }
    
    public static void UpperKickerReverse(double speed) {
        upperKickerController.set(-speed);
    }

    public static void UpperKickerStop() {
        upperKickerController.stopMotor();
    }
    
    public static void LowerKickerRun(double speed) {
        lowerKickerController.set(speed);
    }

    public static void LowerKickerReverse(double speed) {
        lowerKickerController.set(-speed);
    }

    public static void LowerKickerStop() {
        lowerKickerController.stopMotor();
    }
}