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
    public final static CANSparkMax leftShooterController = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    final CANSparkMax rightShooterController = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    public final static CANSparkMax kickerController = new CANSparkMax(ShooterConstants.KICKER_CONTROLLER_PORT, MotorType.kBrushless);
    

    // Shooter motor controller encoders
    final RelativeEncoder leftShooterControllerEncoder = leftShooterController.getEncoder();
    final RelativeEncoder rightShooterControllerEncoder = rightShooterController.getEncoder();
    final RelativeEncoder kickerControllerEncoder = kickerController.getEncoder();
	public static final double ShooterRPM = 0;


    public double velocity = 0;
    public static double speed = leftShooterController.getAppliedOutput();

    // Creates the ShooterSubsystem
    public ShooterSystem() {
        leftShooterController.restoreFactoryDefaults();
        leftShooterController.setIdleMode(IdleMode.kCoast);
        leftShooterController.setInverted(true);

        rightShooterController.restoreFactoryDefaults();
        rightShooterController.setIdleMode(IdleMode.kCoast);
        rightShooterController.follow(leftShooterController, true);


        leftShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        leftShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        rightShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        rightShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);


        Shuffleboard.getTab("Shooting").add("Shooter Control", ShooterConstants.MAX_INPUT)
                .withWidget(BuiltInWidgets.kNumberSlider).getEntry().addListener(event -> {
                    ShooterConstants.MAX_INPUT = event.value.getDouble();
                }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
    }

    public void periodic() {
        SmartDashboard.putNumber("desiredVelocity", velocity);
        SmartDashboard.putNumber("actualVelocity", speed);

    }

    public static double verifyVelocity(double speed) {
        int sign = (int) (speed / Math.abs(speed));
        if (Math.abs(speed) > ShooterConstants.MAX_INPUT)
            return sign * ShooterConstants.MAX_INPUT;
        return speed;
    }

    public static void ShooterRun(double rpm) {
        SmartDashboard.putNumber("Flywheel Set Velocity", rpm);
    }

    public static void ShooterStop() {
        SmartDashboard.putNumber("Flywheel Set Velocity", 0);
    }

    public void ShooterReverse(double rpm) {
        SmartDashboard.putNumber("Flywheel Set Velocity", -rpm);
    }

    public double InchesToMeters(double inches) {
        return inches / 39.37;
    }

    public static void KickerRun(double speed) {
        kickerController.set(speed);
    }
    
    public static void KickerStop() {
        kickerController.set(0);
    }
}