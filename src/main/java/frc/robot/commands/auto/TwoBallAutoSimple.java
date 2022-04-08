package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PositioningConstants;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class TwoBallAutoSimple extends SequentialCommandGroup {
    public TwoBallAutoSimple(DriveSystem driveSystem, IntakeSystem intakeSystem, FieldPositioningSystem positioningSystem, ShooterSystem shooterSystem, double distanceFromBall) {
        
        addCommands(
            new AutoIntakeDownBackward(driveSystem, positioningSystem),
            new AutoShoot(intakeSystem, shooterSystem),
            new RotateToAngle(driveSystem, positioningSystem, 165, .45),
            new Wait(.25),
            new ParallelRaceGroup(
                new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall + IntakeConstants.INTAKE_LENGTH, .5),
                new AutoIntake(intakeSystem, shooterSystem)
            ),
            new Wait(.25),
            new RotateToAngle(driveSystem, positioningSystem, 165, .45),
            new Wait(.25),
            new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall, .45),
            new AutoShoot(intakeSystem, shooterSystem)
        );

    }
    
}
