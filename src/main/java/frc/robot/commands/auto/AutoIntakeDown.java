package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeDown extends SequentialCommandGroup {
    public AutoIntakeDown(DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem) {
        addCommands(
            new MoveXInches(driveSystem, fieldPositioningSystem, 20, .5),
            new MoveXInches(driveSystem, fieldPositioningSystem, 5, -.5)

        );
    }

}
