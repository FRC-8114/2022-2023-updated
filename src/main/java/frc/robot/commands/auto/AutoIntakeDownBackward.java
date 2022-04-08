package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeDownBackward extends SequentialCommandGroup {
    public AutoIntakeDownBackward(DriveSystem driveSystem, FieldPositioningSystem positioningSystem) {
        addCommands(
            //new MoveXInchesBackwards(driveSystem, positioningSystem, 40, .7),
            //new Wait(.5),
            //new MoveXInchesForward(driveSystem, positioningSystem, 20, .5)

            new MoveXInchesBackwards(driveSystem, positioningSystem, 15, .7)

        );
    }

}
