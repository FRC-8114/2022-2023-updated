package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ClimberSystem extends SubsystemBase {
    final CANSparkMax climberController = new CANSparkMax(21, MotorType.kBrushed);

    //Creates the IntakeSubsystem
    public ClimberSystem() {

        //set to factory default and idle so we know what we're working with 
        climberController.restoreFactoryDefaults();
        climberController.setInverted(true);
        climberController.set(0);
    }

    public void ClimberUp(double speed) {
        climberController.set(speed);
    }

    public void ClimberDown(double speed) {
        climberController.set(-speed);
    }

    public void ClimberStop() {
        climberController.set(0);
    }
}
