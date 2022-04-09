package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

public class ShootFromStart extends ParallelCommandGroup {
    public ShootFromStart(DriveSystem driveSystem, FieldPositioningSystem positioningSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        addCommands(
            new MoveXInchesBackwards(driveSystem, positioningSystem, 15, .7),
            new AutoShoot(intakeSystem, shooterSystem)

        );

    }
    
}
