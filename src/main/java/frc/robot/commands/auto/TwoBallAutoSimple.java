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
    public TwoBallAutoSimple(DriveSystem driveSystem, IntakeSystem intakeSystem, FieldPositioningSystem positioningSystem, ShooterSystem shooterSystem, double angleOfNet, double distanceFromBall) {
        
        addCommands(
            new AutoIntakeDownForward(driveSystem, positioningSystem),
            new Wait(.5),
            new ParallelRaceGroup(
                new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall + IntakeConstants.INTAKE_LENGTH, .5),
                new AutoIntake(intakeSystem, shooterSystem)

            ),
            new Rotate2(driveSystem, 180, .5),
            new MoveXInchesForward(driveSystem, positioningSystem, distanceFromBall, .45),
            new AutoShoot(intakeSystem, shooterSystem)
            //new RotateToAngle(driveSystem, positioningSystem, angleOfNet, 1, .2)

        );

    }
    
}
