package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.Constants.ControlConstants;

public class AutoIntake extends CommandBase {
    private IntakeSystem intakeSystem;

    final private double intakeRunSpeed = ControlConstants.Auto.INTAKE_INITIAL_RUN_SPEED;
    
    public AutoIntake (IntakeSystem intakeSystem) {
        this.intakeSystem = intakeSystem;

    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        intakeSystem.IntakeRun(intakeRunSpeed);
        
    }
    @Override
    //Ends when interrupted
    public void end(boolean interrupted) {
        intakeSystem.IntakeStop();
        
    }
    
}
