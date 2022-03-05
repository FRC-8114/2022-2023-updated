package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.Constants.ControlConstants;

public class AutoIntake extends CommandBase {
    private IntakeSystem intakeSystem;
    private ShooterSystem shooterSystem;
    private Timer timer;

    final private static double intakeRunSpeed = ControlConstants.INTAKE_INITIAL_REVERSE_SPEED;
    final private static double upperKickerRunSpeed = ControlConstants.UPPER_KICKER_INITIAL_RUN_SPEED;
    final private static double lowerKickerReverseSpeed = ControlConstants.LOWER_KICKER_INITIAL_REVERSE_SPEED;
    public AutoIntake (IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        this.intakeSystem = intakeSystem;
        this.shooterSystem = shooterSystem;

    }
    public void initialize() {
        timer.start();

    }
    public void execute() {
        if (timer.get() < 1) {
            intakeSystem.IntakeRun(intakeRunSpeed);
            shooterSystem.UpperKickerRun(upperKickerRunSpeed);

        }
        else {
            intakeSystem.IntakeStop();
            shooterSystem.UpperKickerStop();

        }
        shooterSystem.LowerKickerReverse(lowerKickerReverseSpeed);

    }
    public void end() {
        shooterSystem.LowerKickerStop();
        timer.reset();

    }
    public boolean isFinished() {
        if (timer.get() >= 4.1)
            return true;
        return false;

    }
    
}
