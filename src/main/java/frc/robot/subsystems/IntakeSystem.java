package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
    // Creates the intake motor controller
    final static CANSparkMax intakeController = new CANSparkMax(IntakeConstants.INTAKE_RUN_PORT, MotorType.kBrushless);

    // Creates the intake motor controller encoder
    final RelativeEncoder intakeControllerEncoder = intakeController.getEncoder();

    //Creates the IntakeSubsystem
    public IntakeSystem () {

        //set to factory default and idle so we know what we're working with 
        intakeController.restoreFactoryDefaults();
        intakeController.setIdleMode(IdleMode.kCoast);
        intakeController.setInverted(Constants.IntakeConstants.INTAKE_RUN_INVERSED);

    }

    // Runs the intake at a speed from 0 to 1.0
    public void IntakeRun (double speed) {
        intakeController.set(speed);
    }

    // Runs the intake at a given voltage
    public void IntakeRunVoltage (double voltage) {
        intakeController.setVoltage(voltage);
    }

    // Reverses the intake at a speed from 0 to 1.0
    public void IntakeReverse (double speed) {
        intakeController.set(-speed);
    }

    // Reverses the intake at a given voltage
    public void IntakeReverseVoltage (double voltage) {
        intakeController.setVoltage(-voltage);
    }

    // Stops all motion on the intake motor
    public void IntakeStop() {
        intakeController.setVoltage(0);
    }
}