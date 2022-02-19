package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
    //the intake motor controller for running and for deploying
    final static CANSparkMax intakeRunController = new CANSparkMax(IntakeConstants.INTAKE_RUN_PORT, MotorType.kBrushless);

    //the intake motor controller encoders
    final RelativeEncoder intakeRunControllerEncoder = intakeRunController.getEncoder();

    //Creates the IntakeSubsystem
    public IntakeSystem() {

        //set to factory default and idle so we know what we're working with 
        intakeRunController.restoreFactoryDefaults();
        intakeRunController.setIdleMode(IdleMode.kCoast);
        intakeRunController.setInverted(Constants.IntakeConstants.INTAKE_RUN_INVERSED);

        intakeDeployController.restoreFactoryDefaults();
        intakeDeployController.setIdleMode(IdleMode.kBrake);
        intakeDeployController.setInverted(Constants.IntakeConstants.INTAKE_DEPLOY_INVERSED);

    }

    public static void IntakeRunnerRun(double speed) {
        intakeRunController.set(speed);
    }

    public void IntakeRunnerReverse(double speed) {
        intakeRunController.set(-speed);
    }

    public static void IntakeRunnerStop() {
        intakeRunController.stopMotor();
    }
}