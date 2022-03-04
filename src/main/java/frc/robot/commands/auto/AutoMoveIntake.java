package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;

public class AutoMoveIntake extends ParallelCommandGroup {
    public AutoMoveIntake (DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, IntakeSystem intakeSystem) {
        /*
        addCommands(
            new AutoIntake(intakeSystem),
            new MoveXInches(driveSystem, fieldPositioningSystem, 20, .2),
            
        );*/
    }
    
}
