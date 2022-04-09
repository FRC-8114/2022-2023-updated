package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ThreeBallAutoAlone extends SequentialCommandGroup{
    public ThreeBallAutoAlone(DriveSystem driveSystem, FieldPositioningSystem positioningSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        addCommands(
            //Copy TwoBallAutoSimpleAlone up until RotateToAngle back to goal
            //and figure out what distanceToBall is set to in RobotContainer
            new ShootFromStart(driveSystem, positioningSystem, intakeSystem, shooterSystem),
            new RotateToAngle(driveSystem, positioningSystem, 164, .6),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new MoveXInchesForward(driveSystem, positioningSystem, 64, .5),
                    new Wait(.1),
                    new MoveXInchesBackwards(driveSystem, positioningSystem, 10, 0.7),
                    new RotateToAngle(driveSystem, positioningSystem, 164, .6)
                ),
                new AutoIntake(intakeSystem)
            ),
            new Wait(.25),
            //Rotate about 100 degrees to the right
            new RotateToAngle(driveSystem, positioningSystem, 260, .45),
            new Wait(.25),
            //Copy moving forward and intaking from two ball auto (might be a different distance)
            new ParallelRaceGroup(
                new MoveXInchesForward(driveSystem, positioningSystem, 65, .5), //second link
                new AutoIntake(intakeSystem)
            ),
            new Wait(.25),
            //Move backward same amount as moving forward
            new MoveXInchesBackwards(driveSystem, positioningSystem, 65, .5), //second link
            new Wait(.25),
            //Rotate about 80 degrees to the right
            new RotateToAngle(driveSystem, positioningSystem, 280, .45),
            new Wait(.25),
            //Move forward same amount as TwoBallAutoAlone
            new MoveXInchesForward(driveSystem, positioningSystem, 64, .5), //first link
            //Auto Shoot
            new AutoShoot(intakeSystem, shooterSystem)

        );

    }
    
}
