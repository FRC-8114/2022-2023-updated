package frc.robot;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.*;
import frc.robot.commands.shooter.*;
import frc.robot.Constants.ControlConstants.TeleOp;
import frc.robot.subsystems.*;

public class RobotContainer {
  public DriveSystem m_driveSystem = new DriveSystem();
  public FieldPositioningSystem positioningSystem;
  public ShooterSystem shooterSystem = new ShooterSystem();
  public IntakeSystem intakeSystem = new IntakeSystem();
  public ClimberSystem climberSystem = new ClimberSystem();
  public BallTrackingSystem ballSystem = new BallTrackingSystem(74, Math.sqrt(Math.pow(640, 2) + Math.pow(480, 2)));

  public XboxController controller = new XboxController(0);

  private int oldLeftTriggerAxis, oldRightTriggerAxis, oldPOV;

  public final double lowerKickerRunSpeed = TeleOp.LOWER_KICKER_INITIAL_RUN_SPEED;
  public final double lowerKickerReverseSpeed = TeleOp.LOWER_KICKER_INITIAL_REVERSE_SPEED;
  public final double upperKickerRunSpeed = TeleOp.UPPER_KICKER_INITIAL_RUN_SPEED;
  public final double upperKickerReverseSpeed = TeleOp.UPPER_KICKER_INITIAL_REVERSE_SPEED;
  public final double intakeRunSpeed = TeleOp.INTAKE_INITIAL_RUN_SPEED;
  public final double intakeReverseSpeed = TeleOp.INTAKE_INITIAL_REVERSE_SPEED;
  public final double climberRunnerRunSpeed = TeleOp.CLIMBER_RUNNER_INITIAL_RUN_SPEED;
  public final double climberRunnerReverseSpeed = TeleOp.CLIMBER_RUNNER_INITIAL_REVERSE_SPEED;
  public final double climberDeployerRunSpeed = TeleOp.CLIMBER_DEPLOYER_INITIAL_RUN_SPEED;
  public final double climberDeployerReverseSpeed = TeleOp.CLIMBER_DEPLOYER_INITIAL_REVERSE_SPEED;

  private boolean oldRightStickButton;

  //CURRENTLY OBSOLETE
  /*
  private double[] startPosition;
  private double[] almostStartPosition;
  private double[] ballPosition;
  private double[] almostBallPosition;
  
  private final int startLocation = 1;
  private final boolean complexAuto = false;
  */

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    configureButtonBindings();

    initializeInstanceVariables();
    sendControlVariableSettersToShuffleboard();

    //CURRENTLY OBSOLETE
    /*
    //Initializes field positioning system with the correct start position and angle
    almostStartPosition = new double[2];
    almostBallPosition = new double[2];
    startPosition = new double[2];
    ballPosition = new double[2];
    if (complexAuto) {
      double angle = 0;
      switch (startLocation) {
        case 1: 
          startPosition = Constants.PositioningConstants.SPAWN_ONE;
          ballPosition = Constants.PositioningConstants.BALL_ONE; 
          angle = Math.PI + Math.atan((ballPosition[1] - startPosition[1]) / (ballPosition[0] - startPosition[0]));
          break;
        case 2: 
          startPosition = Constants.PositioningConstants.SPAWN_TWO; 
          ballPosition = Constants.PositioningConstants.BALL_TWO; 
          angle = -Math.PI + Math.atan((ballPosition[1] - startPosition[1]) / (ballPosition[0] - startPosition[0]));
          break;
        case 3: 
          startPosition = Constants.PositioningConstants.SPAWN_THREE;
          ballPosition = Constants.PositioningConstants.BALL_THREE; 
          angle = -Math.PI + Math.atan((ballPosition[1] - startPosition[1]) / (ballPosition[0] - startPosition[0]));
          break;
        default:
          angle = Math.PI;

      }

      positioningSystem = new FieldPositioningSystem(m_driveSystem, startPosition, angle);
      almostStartPosition[0] = startPosition[0] + 1.5 * Constants.IntakeConstants.INTAKE_LENGTH * Math.cos(angle);
      almostStartPosition[1] = startPosition[1] + 1.5 * Constants.IntakeConstants.INTAKE_LENGTH  * Math.sin(angle);
      almostBallPosition[0] = ballPosition[0] - 2 * Constants.BALL_RADIUS * Math.cos(angle);
      almostBallPosition[1] = ballPosition[1] - 2 * Constants.BALL_RADIUS * Math.sin(angle);
      positioningSystem = new FieldPositioningSystem(m_driveSystem);
    
    }
    else
      positioningSystem = new FieldPositioningSystem(m_driveSystem);
    */

  }

  /**
   * Initializes the control variables using the constants in ControlConstants
   */
  public void initializeInstanceVariables() {
    oldLeftTriggerAxis = oldRightTriggerAxis = 0;
    oldPOV = -1;
    oldRightStickButton = false;
    
  }

  public void sendControlVariableSettersToShuffleboard() {
    try {
      RobotContainer robotContainer = this;

      Method lowerKickerRunSpeedSetter = RobotContainer.class.getMethod("setLowerKickerRunSpeed", Double.class);
      Method lowerKickerReverseSpeedSetter = RobotContainer.class.getMethod("setLowerKickerReverseSpeed", Double.class);
      Method upperKickerRunSpeedSetter = RobotContainer.class.getMethod("setUpperKickerRunSpeed", Double.class);
      Method upperKickerReverseSpeedSetter = RobotContainer.class.getMethod("setUpperKickerReverseSpeed", Double.class);
      Method intakeRunSpeedSetter = RobotContainer.class.getMethod("setIntakeRunSpeed", Double.class);
      Method intakeReverseSpeedSetter = RobotContainer.class.getMethod("setIntakeReverseSpeed", Double.class);
      Method maxDriveInputSetter = DriveSystem.class.getMethod("setMaxInput", Double.class);
      Method climberDeployerRunSpeedSetter = RobotContainer.class.getMethod("setClimberDeployerRunSpeed", Double.class);
      Method climberDeployerReverseSpeedSetter = RobotContainer.class.getMethod("setClimberDeployerReverseSpeed", Double.class);

      RobotUtils.sendSetterToShuffleboard(robotContainer, lowerKickerRunSpeedSetter, "Control Variables", "lowerKickerRunSpeed", lowerKickerRunSpeed);
      RobotUtils.sendSetterToShuffleboard(robotContainer, lowerKickerReverseSpeedSetter, "Control Variables", "lowerKickerReverseSpeed", lowerKickerReverseSpeed);
      RobotUtils.sendSetterToShuffleboard(robotContainer, upperKickerRunSpeedSetter, "Control Variables", "upperKickerRunSpeed", upperKickerRunSpeed);
      RobotUtils.sendSetterToShuffleboard(robotContainer, upperKickerReverseSpeedSetter, "Control Variables", "upperKickerReverseSpeed", upperKickerReverseSpeed);
      RobotUtils.sendSetterToShuffleboard(robotContainer, intakeRunSpeedSetter, "Control Variables", "intakeRunSpeed", intakeRunSpeed);
      RobotUtils.sendSetterToShuffleboard(robotContainer, intakeReverseSpeedSetter, "Control Variables", "intakeReverseSpeed", intakeReverseSpeed);
      RobotUtils.sendSetterToShuffleboard(m_driveSystem, maxDriveInputSetter, "Control Variables", "maxDriveInput", Constants.DriveConstants.INITIAL_MAX_INPUT);
      RobotUtils.sendSetterToShuffleboard(robotContainer, climberDeployerRunSpeedSetter, "Control Variables", "setClimberDeployerRunSpeed", climberDeployerRunSpeed);
      RobotUtils.sendSetterToShuffleboard(robotContainer, climberDeployerReverseSpeedSetter, "Control Variables", "setClimberDeployerReverseSpeed", climberDeployerReverseSpeed);
    } catch (NoSuchMethodException | SecurityException e) {
      SmartDashboard.putString("Number Setter Sender Error", e.toString());
    }

  }

  public void sendMotorInversionsToShuffleboard() {
    try {
      Method setClimberRunInverted = ClimberSystem.class.getMethod("setClimberRunInverted", boolean.class);
      Method setClimberDeployInverted = ClimberSystem.class.getMethod("setClimberDeployInverted", boolean.class);
      Method setIntakeInverted = IntakeSystem.class.getMethod("setIntakeInverted", boolean.class);
      Method setShooterInverted = ShooterSystem.class.getMethod("setClimberRunInverted", boolean.class);
      Method setLowerKickerInverted = ShooterSystem.class.getMethod("setLowerKickerInverted", boolean.class);
      Method setUpperKickerInverted = ShooterSystem.class.getMethod("setUpperKickerInverted", boolean.class);

      RobotUtils.sendSetterToShuffleboard(climberSystem, setClimberRunInverted, "Motor Inversions", "setClimberRunInverted", Constants.ClimberConstants.CLIMBER_RUN_INVERSED);
      RobotUtils.sendSetterToShuffleboard(climberSystem, setClimberDeployInverted, "Motor Inversions", "setClimberDeployInverted", Constants.ClimberConstants.CLIMBER_DEPLOY_INVERSED);
      RobotUtils.sendSetterToShuffleboard(intakeSystem, setIntakeInverted, "Motor Inversions", "setIntakeInverted", Constants.IntakeConstants.INTAKE_RUN_INVERSED);
      RobotUtils.sendSetterToShuffleboard(shooterSystem, setShooterInverted, "Motor Inversions", "setShooterInverted", Constants.ShooterConstants.SHOOTER_INVERSED);
      RobotUtils.sendSetterToShuffleboard(shooterSystem, setLowerKickerInverted, "Motor Inversions", "setLowerKickerInverted", Constants.ShooterConstants.LOWER_KICKER_INVERSED);
      RobotUtils.sendSetterToShuffleboard(shooterSystem, setUpperKickerInverted, "Motor Inversions", "setUpperKickerInverted", Constants.ShooterConstants.UPPER_KICKER_INVERSED);
    } catch (NoSuchMethodException | SecurityException e) {
      SmartDashboard.putString("Boolean Setter Sender Error", e.toString());
    }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //buttons
    //lower kicker reverse (A)
    new JoystickButton(controller, Button.kA.value)
      .whileHeld(() -> shooterSystem.LowerKickerReverse(lowerKickerReverseSpeed))
      .whenReleased(() -> shooterSystem.LowerKickerStop());
    //all kicker reverse (B)
    new JoystickButton(controller, Button.kB.value) 
      .whileHeld(() -> new AllKickerReverse(upperKickerReverseSpeed, lowerKickerReverseSpeed, shooterSystem).schedule())
      .whenReleased(() -> new AllKickerStop(shooterSystem).schedule());
    //shooter reverse (X)
    new JoystickButton(controller, Button.kX.value)
      .whileHeld(() -> shooterSystem.ShooterReverseVoltage(7))
      .whenReleased(() -> shooterSystem.ShooterStop());
    // Run Kickers forward (Y)
    new JoystickButton(controller, Button.kY.value)
      .whenPressed(() -> new AllKickerRun(TeleOp.UPPER_KICKER_INITIAL_RUN_SPEED, TeleOp.LOWER_KICKER_INITIAL_RUN_SPEED, shooterSystem).schedule())
      .whenReleased(() -> new AllKickerStop(shooterSystem).schedule());
    /*/ Spin button (Y)
    new JoystickButton(controller, Button.kY.value)
      .whenPressed(() -> getDriveSystem().arcadeDrive(0, 90))
      .whenReleased(() -> getDriveSystem().arcadeDrive(0, 0));
    */
    //bumpers
    //intake reverse (LB)
    new JoystickButton(controller, Button.kLeftBumper.value)
      .whileHeld(() -> intakeSystem.IntakeReverse(intakeReverseSpeed))
      .whenReleased(() -> intakeSystem.IntakeStop());

  }

  public XboxController getXboxController() {
    return controller;
  }

  public DriveSystem getDriveSystem() {
    return m_driveSystem;
  }

  public void periodic() {
    //triggers
    //intake and kickers (LT)
    if(controller.getLeftTriggerAxis() == 1) {
      intakeSystem.IntakeRun(intakeRunSpeed);
      shooterSystem.UpperKickerRun(upperKickerRunSpeed);
      shooterSystem.LowerKickerReverse(lowerKickerReverseSpeed);

    }
    else if (oldLeftTriggerAxis == 1) {
      intakeSystem.IntakeStop();
      shooterSystem.UpperKickerStop();
      shooterSystem.LowerKickerStop();

    }
    //auto shoot (RT)
    if(controller.getRightTriggerAxis() == 1) {
      shooterSystem.ShooterRunVoltage(7);
      //shooterSystem.runShooterAt(2100);
      if (shooterSystem.ShooterRPM >= 2450 && shooterSystem.ShooterRPM <= 2600) {
        shooterSystem.LowerKickerRun(lowerKickerRunSpeed);
        shooterSystem.UpperKickerRun(upperKickerRunSpeed);
      } 
    } else if (oldRightTriggerAxis == 1) {
      shooterSystem.LowerKickerStop();
      shooterSystem.UpperKickerStop();
      shooterSystem.ShooterStop();
    }

    //d-pad
    switch (controller.getPOV()) {
      case 0: // UP
        climberSystem.ClimberRunnerUp(climberRunnerRunSpeed);
        break;
      case 90: // RIGHT
        climberSystem.ClimberDeployerDown(climberDeployerRunSpeed);
        break;
      case 180: // DOWN
        climberSystem.ClimberRunnerDown(climberRunnerReverseSpeed);
        break;
      case 270: // LEFT
        climberSystem.ClimberDeployerUp(climberDeployerReverseSpeed);
        break;
      default: // NONE
        if (oldPOV >= 0) {
          climberSystem.ClimberStop();
          climberSystem.ClimberDeployerStop();
        }
    }

    //sticks
    //reverse drive (RS)
    if (!oldRightStickButton && controller.getRightStickButton())
      m_driveSystem.switchMotorPorts();
    
    //old inputs
    oldLeftTriggerAxis = (int)controller.getLeftTriggerAxis();
    oldRightTriggerAxis = (int)controller.getRightTriggerAxis();
    oldRightStickButton = controller.getRightStickButton();
    oldPOV = ((int)controller.getPOV());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Ro
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *bot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    positioningSystem.zeroPosition();

    //return new OneBallAuto(m_driveSystem, positioningSystem, intakeSystem, shooterSystem);
    //return new TwoBallAutoSimpleWall(m_driveSystem, intakeSystem, positioningSystem, shooterSystem, 35);
    //return new TwoBallAutoSimpleMiddle(m_driveSystem, intakeSystem, positioningSystem, shooterSystem, 35);
    //return new TwoBallAutoSimpleHangar(m_driveSystem, intakeSystem, positioningSystem, shooterSystem, 35);
    //return new RotateToAngle(m_driveSystem, positioningSystem, 165, .6); 
    return new AutoSpin(m_driveSystem, positioningSystem, intakeSystem, shooterSystem);

  }

}
