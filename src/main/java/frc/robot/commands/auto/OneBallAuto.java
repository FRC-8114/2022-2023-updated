package frc.robot.commands.auto;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.ShooterSystem;

public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(DriveSystem driveSystem, FieldPositioningSystem positioningSystem, ShooterSystem shooterSystem) {
        addCommands(
            new AutoIntakeDown(driveSystem, positioningSystem),
            new AutoShoot(3700, shooterSystem),
            new MoveXInchesBackwards(driveSystem, positioningSystem, 100, .4)

        );

    }
    
}
