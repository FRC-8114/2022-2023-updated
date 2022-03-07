package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.Constants.ControlConstants;

public class AutoIntake extends CommandBase {
    private IntakeSystem intakeSystem;
    private ShooterSystem shooterSystem;

    final private static double intakeRunSpeed = ControlConstants.INTAKE_INITIAL_REVERSE_SPEED;
    final private static double upperKickerRunSpeed = ControlConstants.UPPER_KICKER_INITIAL_RUN_SPEED;
    final private static double lowerKickerReverseSpeed = ControlConstants.LOWER_KICKER_INITIAL_REVERSE_SPEED;
    public AutoIntake (IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        this.intakeSystem = intakeSystem;
        this.shooterSystem = shooterSystem;

    }
    public void initialize() {

    }
    public void execute() {
        intakeSystem.IntakeRun(intakeRunSpeed);
        shooterSystem.UpperKickerRun(upperKickerRunSpeed);
        shooterSystem.LowerKickerReverse(lowerKickerReverseSpeed);

    }
    public void end(boolean interrupted) {
        intakeSystem.IntakeStop();
        shooterSystem.UpperKickerStop();
        shooterSystem.LowerKickerStop();

    }
    public boolean isFinished() {
        return false;

    }
    
}
