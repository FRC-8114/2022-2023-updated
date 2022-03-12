package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTrackingSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class TwoBallAutoComplex extends SequentialCommandGroup {
    public TwoBallAutoComplex (BallTrackingSystem ballSystem, DriveSystem driveSystem, FieldPositioningSystem positioningSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem, double[] almostBallPosition, double[] almostStartPosition) {
        addCommands(
            new AutoIntakeDownBackward(driveSystem, positioningSystem),
            new AutoShoot(intakeSystem, shooterSystem),
            new MoveToPosition(driveSystem, positioningSystem, almostBallPosition),
            new RotateToBall(ballSystem, positioningSystem, driveSystem),
            new ParallelRaceGroup(
                new MoveXInchesForward(driveSystem, positioningSystem, 20, .3),
                new AutoIntake(intakeSystem, shooterSystem)

            ),
            new MoveToPosition(driveSystem, positioningSystem, almostBallPosition),
            new MoveToPosition(driveSystem, positioningSystem, almostStartPosition),
            new AutoShoot(intakeSystem, shooterSystem)

        );

    }
    
}
