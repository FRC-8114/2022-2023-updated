package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ClimberSystem extends SubsystemBase {
    final CANSparkMax climberRunController = new CANSparkMax(ClimberConstants.CLIMBER_RUN_PORT, MotorType.kBrushed);
    final CANSparkMax climberDeployController = new CANSparkMax(ClimberConstants.CLIMBER_DEPLOY_PORT,
            MotorType.kBrushless);

    final RelativeEncoder climberRunControllerEncoder = climberRunController.getEncoder();
    final RelativeEncoder climberDeployControllerEncoder = climberDeployController.getEncoder();

    // Creates the IntakeSubsystem
    public ClimberSystem() {

        // set to factory default and idle so we know what we're working with
        climberRunController.restoreFactoryDefaults();
        climberRunController.setInverted(false);
        climberRunController.setIdleMode(IdleMode.kBrake);
        climberRunController.stopMotor();

        climberDeployController.restoreFactoryDefaults();
        climberDeployController.setInverted(false);
        climberDeployController.setIdleMode(IdleMode.kBrake);
        climberDeployController.stopMotor();
    }

    public void ClimberRunnerUp(double speed) {
        climberRunController.set(speed);
    }

    public void ClimberRunnerDown(double speed) {
        climberRunController.set(-speed);
    }

    public void ClimberStop() {
        climberRunController.stopMotor();
    }

    public void ClimberDeployerUp(double speed) {
        climberDeployController.set(speed);
    }

    public void ClimberDeployerDown(double speed) {
        climberDeployController.set(-speed);
    }

    public void ClimberDeployerStop() {
        climberDeployController.stopMotor();
    }
}
