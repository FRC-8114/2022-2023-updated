package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotUtils;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(DriveSystem drive, FieldPositioningSystem fieldPositioning, double[] desiredPos) {
        RobotUtils.sendToShuffleboard("xDist", fieldPositioning.xDistanceFrom(desiredPos));
        RobotUtils.sendToShuffleboard("yDist", fieldPositioning.yDistanceFrom(desiredPos));

        addCommands(
            new RotateToAngle(drive, fieldPositioning, fieldPositioning.angleToPoint(desiredPos), 0.3),

            new MoveXInchesForward(drive, fieldPositioning, fieldPositioning.distanceFrom(desiredPos), 0.4)
        );
    }
}
