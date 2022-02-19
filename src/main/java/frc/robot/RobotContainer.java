package frc.robot;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public IntakeSystem intakeSystem = new IntakeSystem();

  public XboxController controller = new XboxController(0);

  public MoveXInches autoCommand = new MoveXInches(m_driveSystem, positioningSystem, 180, -0.2);

  public double lowerKickerRunSpeed, lowerKickerReverseSpeed;
  public double upperKickerRunSpeed, upperKickerReverseSpeed;
  public double shooterRunSpeed, shooterReverseSpeed;
  public double intakeRunSpeed, intakeReverseSpeed;

  private int oldLeftTriggerAxis;
  private int oldRightTriggerAxis;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    configureButtonBindings();

    initializeControlVariables();
    sendControlVariableSettersToShuffleboard();
  }

  /**
   * Initializes the control variables using the constants in ControlConstants
   */
  public void initializeControlVariables() {
    lowerKickerRunSpeed = Constants.ControlConstants.LOWER_KICKER_INITIAL_RUN_SPEED;
    lowerKickerReverseSpeed = Constants.ControlConstants.LOWER_KICKER_INITIAL_REVERSE_SPEED;
    upperKickerRunSpeed = Constants.ControlConstants.UPPER_KICKER_INITIAL_RUN_SPEED;
    upperKickerReverseSpeed = Constants.ControlConstants.UPPER_KICKER_INITIAL_REVERSE_SPEED;
    shooterRunSpeed = Constants.ControlConstants.SHOOTER_INITIAL_RUN_SPEED;
    shooterReverseSpeed = Constants.ControlConstants.SHOOTER_INITIAL_REVERSE_SPEED;
    intakeRunSpeed = Constants.ControlConstants.INTAKE_INITIAL_RUN_SPEED;
    intakeReverseSpeed = Constants.ControlConstants.INTAKE_INITIAL_REVERSE_SPEED;
  }

  public void sendControlVariableSettersToShuffleboard() {
    try {
      RobotContainer robotContainer = this;

      Method lowerKickerRunSpeedSetter = RobotContainer.class.getMethod("setLowerKickerRunSpeed", Double.class);
      Method lowerKickerReverseSpeedSetter = RobotContainer.class.getMethod("setLowerKickerReverseSpeed", Double.class);
      Method upperKickerRunSpeedSetter = RobotContainer.class.getMethod("setUpperKickerRunSpeed", Double.class);
      Method upperKickerReverseSpeedSetter = RobotContainer.class.getMethod("setUpperKickerReverseSpeed", Double.class);
      Method shooterRunSpeedSetter = RobotContainer.class.getMethod("setShooterRunSpeed", Double.class);
      Method shooterReverseSpeedSetter = RobotContainer.class.getMethod("setShooterReverseSpeed", Double.class);
      Method intakeRunSpeedSetter = RobotContainer.class.getMethod("setIntakeRunSpeed", Double.class);
      Method intakeReverseSpeedSetter = RobotContainer.class.getMethod("setIntakeReverseSpeed", Double.class);

      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, lowerKickerRunSpeedSetter, "Control Variables", "lowerKickerRunSpeed", lowerKickerRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, lowerKickerReverseSpeedSetter, "Control Variables", "lowerKickerReverseSpeed", lowerKickerReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, upperKickerRunSpeedSetter, "Control Variables", "upperKickerRunSpeed", upperKickerRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, upperKickerReverseSpeedSetter, "Control Variables", "upperKickerReverseSpeed", upperKickerReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, shooterRunSpeedSetter, "Control Variables", "shooterRunSpeed", shooterRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, shooterReverseSpeedSetter, "Control Variables", "shooterReverseSpeed", shooterReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, intakeRunSpeedSetter, "Control Variables", "intakeRunSpeed", intakeRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, intakeReverseSpeedSetter, "Control Variables", "intakeReverseSpeed", intakeReverseSpeed);
    } catch (NoSuchMethodException | SecurityException e) {
      SmartDashboard.putString("depressing_error", e.toString());
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller, Button.kA.value)
      .whenPressed(() -> m_driveSystem.switchMotorPorts());

    new JoystickButton(controller, Button.kY.value)
      .whenPressed(() -> shooterSystem.LowerKickerReverse(lowerKickerReverseSpeed))
      .whenReleased(() -> shooterSystem.LowerKickerStop());

    new JoystickButton(controller, 5)
      .whenPressed(() -> shooterSystem.UpperKickerRun(upperKickerRunSpeed))
      .whenReleased(() -> shooterSystem.UpperKickerStop());

    new JoystickButton(controller, 6)
      .whenPressed(() -> intakeSystem.IntakeRunnerReverse(intakeReverseSpeed))
      .whenReleased(() -> intakeSystem.IntakeRunnerStop());


    new JoystickButton(controller, Button.kA.value)
      .whenPressed(() -> m_driveSystem.switchMotorPorts());

    new JoystickButton(controller, Button.kB.value)
      .whenPressed(() -> m_driveSystem.switchDriveSystem());
  }

  public XboxController getXboxController() {
    return controller;
  }

  public DriveSystem getDriveSystem() {
    return m_driveSystem;
  }

  public void periodic() {
    if(controller.getLeftTriggerAxis() == 1) {
      intakeSystem.IntakeRunnerRun(intakeRunSpeed);
      shooterSystem.LowerKickerRun(lowerKickerRunSpeed);
      
    }
    else if (oldLeftTriggerAxis == 1) {
      intakeSystem.IntakeRunnerStop();
      shooterSystem.LowerKickerStop();

    }
    if(controller.getRightTriggerAxis() == 1) {
      shooterSystem.ShooterRun(shooterRunSpeed);

    }
    else if (oldRightTriggerAxis == 1) {
      shooterSystem.ShooterStop();

    }
    oldLeftTriggerAxis = (int)controller.getLeftTriggerAxis();
    oldRightTriggerAxis = (int)controller.getRightTriggerAxis();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }

  /**
   * Setters
   */

  public void setLowerKickerRunSpeed(Double speed) {
    lowerKickerRunSpeed = speed;
  }

  public void setLowerKickerReverseSpeed(Double speed) {
    lowerKickerReverseSpeed = speed;
  }

  public void setUpperKickerRunSpeed(Double speed) {
    upperKickerRunSpeed = speed;
  }

  public void setUpperKickerReverseSpeed(Double speed) {
    upperKickerReverseSpeed = speed;
  }

  public void setShooterRunSpeed(Double speed) {
    shooterRunSpeed = speed;
  }

  public void setShooterReverseSpeed(Double speed) {
    shooterReverseSpeed = speed;
  }

  public void setIntakeRunSpeed(Double speed) {
    intakeRunSpeed = speed;
  }

  public void setIntakeReverseSpeed(Double speed) {
    intakeReverseSpeed = speed;
  }
}
