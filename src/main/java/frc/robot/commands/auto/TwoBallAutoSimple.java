package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class TwoBallAutoSimple extends SequentialCommandGroup {
    public TwoBallAutoSimple(DriveSystem driveSystem, IntakeSystem intakeSystem, FieldPositioningSystem positioningSystem, ShooterSystem shooterSystem, double[] ballPosition) {
        double inchesForward = positioningSystem.distanceFrom(ballPosition) + 10;
        addCommands(
            new ParallelRaceGroup(
                new MoveXInchesForward(driveSystem, positioningSystem, inchesForward, .5),
                new AutoIntake(intakeSystem, shooterSystem)

            ),
            new Rotate(driveSystem, positioningSystem, 180, 1, .3),
            new MoveXInchesForward(driveSystem, positioningSystem, inchesForward, .45),
            new AutoShoot(shooterSystem)

        );

    }
    
}
