package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PositioningConstants;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class TwoBallAutoSimple extends SequentialCommandGroup {
    public TwoBallAutoSimple(DriveSystem driveSystem, IntakeSystem intakeSystem, FieldPositioningSystem positioningSystem, ShooterSystem shooterSystem, int location, double angleOfNet, double distanceFromBall) {
        addCommands(
            new ParallelRaceGroup(
                new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall + 10, .5),
                new AutoIntake(intakeSystem, shooterSystem)

            ),
            new Rotate(driveSystem, positioningSystem, 180, 1, .3),
            new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall, .45),
<<<<<<< HEAD
            new AutoShoot(shooterSystem),
            new RotateToAngle(driveSystem, positioningSystem, angleOfNet, .2)
=======
            new AutoShoot(shooterSystem)
            //new RotateToAngle(driveSystem, positioningSystem, angleOfNet, 1, .2)
>>>>>>> all

        );

    }
    
}
