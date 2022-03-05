package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class AutoMoveIntake extends ParallelCommandGroup {
    public AutoMoveIntake (ShooterSystem shooterSystem, DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, IntakeSystem intakeSystem) {
        addCommands(
            new AutoIntake(intakeSystem, shooterSystem),
            new MoveXInchesBackwards(driveSystem, fieldPositioningSystem, 20, .2)  
        );
    }
    
}
