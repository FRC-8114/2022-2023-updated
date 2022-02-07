package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.MoveXInches;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class RobotContainer {
  public DriveSystem m_driveSystem = new DriveSystem();
  public FieldPositioningSystem positioningSystem = new FieldPositioningSystem(m_driveSystem);
  public ShooterSystem shooterSystem = new ShooterSystem();

  public XboxController controller = new XboxController(0);

  public MoveXInches autoCommand = new MoveXInches(m_driveSystem, positioningSystem, 180, -0.2);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller, 5)
      .whenPressed(() -> shooterSystem.UpperKickerRun(.25))
      .whenReleased(() -> shooterSystem.UpperKickerStop());

    new JoystickButton(controller, 6)
        .whenPressed(() -> shooterSystem.LowerKickerRun(.8))
        .whenReleased(() -> shooterSystem.LowerKickerStop());

    new JoystickButton(controller, Button.kY.value)
      .whenPressed(() -> shooterSystem.LowerKickerReverse(.15))
      .whenReleased(() -> shooterSystem.LowerKickerStop());
  }

  public XboxController getXboxController() {
    return controller;
  }

  public DriveSystem getDriveSystem() {
    return m_driveSystem;
  }

  public void periodic() {
    if(controller.getLeftTriggerAxis() == 1) {
      IntakeSystem.IntakeRunnerRun(.25);
    }
    else {
      IntakeSystem.IntakeRunnerStop();
    }
    if(controller.getRightTriggerAxis() == 1) {
      ShooterSystem.ShooterRun(.80);
    }
    else {
      ShooterSystem.ShooterStop();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
