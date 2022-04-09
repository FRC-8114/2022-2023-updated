package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class AutoShoot extends CommandBase {
    private IntakeSystem intakeSystem;
    private ShooterSystem shooterSystem;
    private Timer timer;
    private int ballCount;

    public AutoShoot(IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        this.intakeSystem = intakeSystem;
        this.shooterSystem = shooterSystem;

        timer = new Timer();
        
    }
    @Override
    public void initialize() {
        timer.start();

    }
    @Override
    public void execute() {
        shooterSystem.runShooterAt(ControlConstants.Auto.SHOOTER_DESIRED_RPM);

        if (shooterSystem.ShooterRPM > ControlConstants.Auto.SHOOTER_DESIRED_RPM - 1) {
            intakeSystem.IntakeRun(ControlConstants.Auto.INTAKE_INITIAL_RUN_SPEED);
            shooterSystem.UpperKickerRun(1);
            shooterSystem.LowerKickerRun(1);
            new Wait(.25);
            ballCount++;
        }
        else {
            intakeSystem.IntakeStop();
            shooterSystem.UpperKickerStop();
            shooterSystem.LowerKickerStop(); 
        }

    }
    @Override
    public void end(boolean interrupted) {
        intakeSystem.IntakeStop();
        shooterSystem.ShooterStop();
        shooterSystem.UpperKickerStop();
        shooterSystem.LowerKickerStop();

    }
    @Override
    public boolean isFinished() {
        if (ballCount >= 1)
            return true;
        return false;

    }

}