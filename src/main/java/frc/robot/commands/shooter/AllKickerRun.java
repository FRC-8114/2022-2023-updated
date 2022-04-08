package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class AllKickerRun extends CommandBase {
    private double upperSpeed;
    private double lowerSpeed;
    private ShooterSystem shooterSystem;

    public AllKickerRun(double upperSpeed, double lowerSpeed, ShooterSystem shooterSystem) {
        this.upperSpeed = upperSpeed;
        this.lowerSpeed = lowerSpeed;
        this.shooterSystem = shooterSystem;

    }
    public void initialize() {
        
    }
    public void execute() {
        shooterSystem.UpperKickerRun(upperSpeed);
        shooterSystem.LowerKickerRun(lowerSpeed);

    }
    public void end() {

    }
    public boolean isFinished() {
        return true;

    }
}
