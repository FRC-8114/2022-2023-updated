package frc.robot.commands.auto;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(DriveSystem driveSystem, FieldPositioningSystem positioningSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        addCommands(
            new AutoIntakeDownBackward(driveSystem, positioningSystem),
            new AutoShoot(intakeSystem, shooterSystem),
            new MoveXInchesBackwards(driveSystem, positioningSystem, 100, .4)

        );

    }
    
}
