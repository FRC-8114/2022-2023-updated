package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoIntakeDownForward extends SequentialCommandGroup {
    public AutoIntakeDownForward(DriveSystem driveSystem, FieldPositioningSystem positioningSystem) {
        addCommands(
            new MoveXInchesForward(driveSystem, positioningSystem, 5, .5),
            new Wait(.5),
            new MoveXInchesBackwards(driveSystem, positioningSystem, 5, .6)

        );
    }

}