package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
    //the intake motor controller
    final CANSparkMax intakeController = new CANSparkMax(IntakeConstants.INTAKE_CONTROLLER_PORT, MotorType.kBrushless);

    //the intake motor controller encoder
    final RelativeEncoder intakeControllerEncoder = intakeController.getEncoder();

    //Creates the IntakeSubsystem
    public IntakeSystem() {

        //set to factory default and idle so we know what we're working with 
        intakeController.restoreFactoryDefaults(); 
        intakeController.setIdleMode(IdleMode.kCoast);

    }

    public void IntakeRun(double speed) {
        intakeController.set(speed);
    }

    public void IntakeReverse(double speed) {
        intakeController.set(-speed);
    }

    public void IntakeStop() {
        intakeController.stopMotor();
    }


}