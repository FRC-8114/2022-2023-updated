package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class AllKickerStop extends CommandBase {
    private ShooterSystem shooterSystem;
    public AllKickerStop(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;

    }
    public void initialize() {
        shooterSystem.LowerKickerStop();
        shooterSystem.UpperKickerStop();

    }
    public void execute() {

    }
    public void end() {

    }
    public boolean isFinished() {
        return true;

    }
    
}
