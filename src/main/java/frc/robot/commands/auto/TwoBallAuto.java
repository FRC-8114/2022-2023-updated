package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class TwoBallAuto extends SequentialCommandGroup {
    private static final int desiredRPM = 4000;
    public TwoBallAuto (DriveSystem driveSystem, FieldPositioningSystem fieldPositioningSystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        /*
        addCommands (
            new AutoShoot (desiredRPM, shooterSystem),
            new MoveToBall (driveSystem, fieldPositioningSystem),
            new AutoMoveIntake (driveSystem, fieldPositioningSystem, intakeSystem)

        );*/

    }
    
}
