package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ClimberSystem extends SubsystemBase {
    final TalonFX climberRunController = new TalonFX(ClimberConstants.CLIMBER_RUN_PORT);
    final CANSparkMax climberDeployController = new CANSparkMax(ClimberConstants.CLIMBER_DEPLOY_PORT,
            MotorType.kBrushless);

    final TalonFX climberRunControllerEncoder = new TalonFX(ClimberConstants.CLIMBER_RUN_PORT);
    final RelativeEncoder climberDeployControllerEncoder = climberDeployController.getEncoder();
    // Creates the IntakeSubsystem
    public ClimberSystem() {

        // set to factory default and idle so we know what we're working with
        climberRunController.configFactoryDefault();
        climberRunController.setInverted(Constants.ClimberConstants.CLIMBER_DEPLOY_INVERSED);
        climberRunController.setNeutralMode(NeutralMode.Brake);

        climberDeployController.restoreFactoryDefaults();
        climberDeployController.setInverted(Constants.ClimberConstants.CLIMBER_DEPLOY_INVERSED);
        climberDeployController.setIdleMode(IdleMode.kBrake);
        climberDeployController.stopMotor();
    }

    public void ClimberRunnerUp(double speed) {
        climberRunController.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void ClimberRunnerDown(double speed) {
        climberRunController.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void ClimberStop() {
        climberRunController.set(TalonFXControlMode.PercentOutput, 0);
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
