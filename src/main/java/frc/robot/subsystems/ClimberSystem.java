package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ClimberSystem extends SubsystemBase {
    // Creates the motor controllers for the dart and the arm
    final WPI_TalonFX climberRunController = new WPI_TalonFX(ClimberConstants.CLIMBER_RUN_PORT);
    final CANSparkMax climberDeployController = new CANSparkMax(ClimberConstants.CLIMBER_DEPLOY_PORT,
            MotorType.kBrushless);

    // Creates a dart encoder
    final RelativeEncoder climberDeployControllerEncoder = climberDeployController.getEncoder();

    // Initializes a potentiometer
    final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0);

    public ClimberSystem() {

        // set to factory default and idle so we know what we're working with
        climberRunController.configFactoryDefault();
        climberRunController.setInverted(Constants.ClimberConstants.CLIMBER_DEPLOY_INVERSED);
        climberRunController.setNeutralMode(NeutralMode.Brake);

        climberDeployControllerEncoder.setPositionConversionFactor(Constants.ClimberConstants.CLIMBER_DEPLOY_CONVERSTION_FACTOR);

        climberDeployController.restoreFactoryDefaults();
        climberDeployController.setInverted(Constants.ClimberConstants.CLIMBER_DEPLOY_INVERSED);
        climberDeployController.setIdleMode(IdleMode.kBrake);
        climberDeployController.stopMotor();

        Shuffleboard.getTab("Deployer velocity").add("Change deployer velocity", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry()
            .addListener(event -> {ClimberDeployerUp(event.value.getDouble());}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
    }

    // Runs the climber arm at a speed from 0 to 1.0
    public void ClimberRunnerUp (double speed) {
        climberRunController.set(speed);
    }

    // Runs the climber arm at a given voltage
    public void ClimberRunnerUpVoltage (double voltage) {
        climberRunController.setVoltage(voltage);
    }

    // Reverses the climber arm at a speed from 0 to 1.0
    public void ClimberRunnerDown (double speed) {
        climberRunController.set(-speed);
    }

    // Reverses the climber arm at a given voltage
    public void ClimberRunnerDownVoltage (double voltage) {
        climberRunController.setVoltage(-voltage);
    }

    // Stops all motion on the arm
    public void ClimberStop() {
        climberRunController.setVoltage(0);
    }

    // Runs the climber dart at a speed from 0 to 1.0
    public void ClimberDeployerUp (double speed) {
        climberDeployController.set(speed);
    }

    // Runs the climber dart at a given voltage
    public void ClimberDeployerUpVoltage (double voltage) {
        climberDeployController.setVoltage(voltage);
    }

    // Reverses the climber dart at a speed from 0 to 1.0
    public void ClimberDeployerDown (double speed) {
        climberDeployController.set(-speed);
    }

    // Reverses the climber dart at a given voltage
    public void ClimberDeployerDownVoltage (double voltage) {
        climberDeployController.setVoltage(-voltage);
    }

    // Stops all motion on the dart
    public void ClimberDeployerStop() {
        climberDeployController.setVoltage(0);
    }

    // Get the current angle of the potentiometer
    public double getPotentiometerAngle() {
        return potentiometer.get();
    }

    // Get the current extended length of the dart
    public double getDartPosition() {
        return 6 + getPotentiometerAngle() * (6/4.4);
    }

    // Setters for shuffleboard
    public void setClimberRunInversed(boolean inverted) {
        climberRunController.setInverted(inverted);
    }

    public void setClimberDeployInversed(boolean inverted) {
        climberDeployController.setInverted(inverted);
    }
}
