package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotUtils;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(DriveSystem drive, FieldPositioningSystem fieldPositioning, double[] desiredPos) {
        RobotUtils.sendToShuffleboard("xDist", fieldPositioning.xDistanceToPointNavx(desiredPos));
        RobotUtils.sendToShuffleboard("yDist", fieldPositioning.yDistanceToPointNavx(desiredPos));

        addCommands(
            new RotateToAngle(drive, fieldPositioning, fieldPositioning.angleToPointNavx(desiredPos), 0.3),

            new MoveXInchesForward(drive, fieldPositioning, fieldPositioning.distanceToPointNavx(desiredPos), 0.4)
        );
    }
}
