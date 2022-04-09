package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.Constants.ControlConstants;

public class AutoIntakeWithKickers extends CommandBase {
    private IntakeSystem intakeSystem;
    private ShooterSystem shooterSystem;
    public AutoIntakeWithKickers(IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        this.intakeSystem = intakeSystem;
        this.shooterSystem = shooterSystem;

    }
    @Override
    public void execute() {
        intakeSystem.IntakeRun(ControlConstants.Auto.INTAKE_INITIAL_RUN_SPEED);
        shooterSystem.UpperKickerRun(ControlConstants.Auto.UPPER_KICKER_INITIAL_RUN_SPEED);
        shooterSystem.LowerKickerReverse(ControlConstants.Auto.LOWER_KICKER_INITIAL_REVERSE_SPEED);

    }
    @Override
    //Ends when interrupted
    public void end(boolean interrupted) {
        intakeSystem.IntakeStop();
        shooterSystem.UpperKickerStop();
        shooterSystem.LowerKickerStop();

    }
    
}
