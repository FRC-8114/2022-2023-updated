package frc.robot.commands.auto;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;

public class TwoBallAutoSimpleHangar extends SequentialCommandGroup{
    public TwoBallAutoSimpleHangar(DriveSystem driveSystem, IntakeSystem intakeSystem, FieldPositioningSystem positioningSystem, ShooterSystem shooterSystem, double distanceFromBall){
        addCommands(
            new ShootFromStart(driveSystem, positioningSystem, intakeSystem, shooterSystem),
            new RotateToAngle(driveSystem, positioningSystem, 164, .6),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall + IntakeConstants.INTAKE_LENGTH, .5),
                    new Wait(.1),
                    new MoveXInchesBackwards(driveSystem, positioningSystem, 10, 0.7),
                    new RotateToAngle(driveSystem, positioningSystem, 164, .6)
                ),
                new AutoIntake(intakeSystem)
            ),
            new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall-5, .45),
            new AutoShoot(intakeSystem, shooterSystem)
        );

    }
    
}
