package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
    //the intake motor controller for running and for deploying
    final CANSparkMax intakeRunController = new CANSparkMax(IntakeConstants.INTAKE_RUN_PORT, MotorType.kBrushless);
    final CANSparkMax intakeDeployController = new CANSparkMax(IntakeConstants.INTAKE_DEPLOY_PORT, MotorType.kBrushless);

    //the intake motor controller encoders
    final RelativeEncoder intakeRunControllerEncoder = intakeRunController.getEncoder();
    final RelativeEncoder intakeDeployControllerEncoder = intakeDeployController.getEncoder();

    //Creates the IntakeSubsystem
    public IntakeSystem() {

        //set to factory default and idle so we know what we're working with 
        intakeRunController.restoreFactoryDefaults();
        intakeRunController.setIdleMode(IdleMode.kCoast);

        intakeDeployController.restoreFactoryDefaults();
        intakeDeployController.setIdleMode(IdleMode.kBrake);

    }

    public void IntakeRunnerRun(double speed) {
        intakeRunController.set(speed);
    }

    public void IntakeRunnerReverse(double speed) {
        intakeRunController.set(-speed);
    }

    public void IntakeRunnerStop() {
        intakeRunController.stopMotor();
    }

    public void IntakeDeployerRunnerUp(double speed) {
        intakeDeployController.set(speed);
    }

    public void IntakeDeployerRunnerDown(double speed) {
        intakeDeployController.set(-speed);
    }

    public void IntakeDeployerRunnerSoftDrop(double time) {
        Timer timer = new Timer();
        timer.start();
        while (timer.get() < time) {
            intakeDeployController.set(.15);
        }
        intakeDeployController.stopMotor();
        timer.stop();
    }

    public void IntakeDeployerRunnerStop() {
        intakeDeployController.stopMotor();
    }
}