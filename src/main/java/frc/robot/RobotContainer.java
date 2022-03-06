package frc.robot;

import java.lang.reflect.Method;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.auto.*;
import frc.robot.commands.shooter.*;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.BallTrackingSystem;
import frc.robot.subsystems.ClimberSystem;

public class RobotContainer {
  public DriveSystem m_driveSystem = new DriveSystem();
  public FieldPositioningSystem positioningSystem;
  public ShooterSystem shooterSystem = new ShooterSystem();
  public IntakeSystem intakeSystem = new IntakeSystem();
  public ClimberSystem climberSystem = new ClimberSystem();
  public BallTrackingSystem ballSystem = new BallTrackingSystem(74, Math.sqrt(Math.pow(640, 2) + Math.pow(480, 2)));

  public XboxController controller = new XboxController(0);

  public double lowerKickerRunSpeed, lowerKickerReverseSpeed;
  public double upperKickerRunSpeed, upperKickerReverseSpeed;
  public double shooterRunSpeed, shooterReverseSpeed;
  public double intakeRunSpeed, intakeReverseSpeed;
  public double autoRotateSpeed;
  public double climberRunnerRunSpeed, climberRunnerReverseSpeed;
  public double climberDeployerRunSpeed, climberDeployerReverseSpeed;
  public double teleopShootSpeed;

  private int oldLeftTriggerAxis, oldRightTriggerAxis, oldPOV;
  private boolean oldRightStickButton;
  
<<<<<<< HEAD
  // final private int start = 1;
  // private double[] startPosition;
  // private double[] almostStartPosition;
  // private double[] ballPosition;
  // private double[] almostBallPosition;
=======
  final private int startLocation = 1;
  private double[] startPosition;
  private double[] almostStartPosition;
  private double[] ballPosition;
  private double[] almostBallPosition;
>>>>>>> all

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    configureButtonBindings();

    initializeControlVariables();
    sendControlVariableSettersToShuffleboard();

    /*
    double angle = Math.PI;
    almostStartPosition = new double[2];
    almostBallPosition = new double[2];

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

    }

    positioningSystem = new FieldPositioningSystem(m_driveSystem, startPosition, angle);
    almostStartPosition[0] = startPosition[0] + 1.5 * Constants.IntakeConstants.INTAKE_LENGTH * Math.cos(angle);
    almostStartPosition[1] = startPosition[1] + 1.5 * Constants.IntakeConstants.INTAKE_LENGTH  * Math.sin(angle);
    almostBallPosition[0] = ballPosition[0] - 2 * Constants.BALL_RADIUS * Math.cos(angle);
    almostBallPosition[1] = ballPosition[1] - 2 * Constants.BALL_RADIUS * Math.sin(angle);
    */
    positioningSystem = new FieldPositioningSystem(m_driveSystem); 

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

    autoRotateSpeed = Constants.AutoConstants.AUTO_ROTATE_SPEED;
    climberRunnerRunSpeed = Constants.ControlConstants.CLIMBER_RUNNER_INITIAL_RUN_SPEED;
    climberRunnerReverseSpeed = Constants.ControlConstants.CLIMBER_RUNNER_INITIAL_REVERSE_SPEED;
    climberDeployerRunSpeed = Constants.ControlConstants.CLIMBER_DEPLOYER_INITIAL_RUN_SPEED;
    climberDeployerReverseSpeed = Constants.ControlConstants.INTAKE_INITIAL_REVERSE_SPEED;
    oldLeftTriggerAxis = oldRightTriggerAxis = 0;
    oldPOV = -1;
    oldRightStickButton = false;
    teleopShootSpeed = 2700;
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
      Method maxDriveInputSetter = DriveSystem.class.getMethod("setMaxInput", Double.class);
      Method autoRotateSpeedSetter = RobotContainer.class.getMethod("setAutoRotateSpeed", Double.class);
      Method teleopShootSpeedSetter = RobotContainer.class.getMethod("teleopShootSpeedSetter", Double.class);

      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, lowerKickerRunSpeedSetter, "Control Variables", "lowerKickerRunSpeed", lowerKickerRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, lowerKickerReverseSpeedSetter, "Control Variables", "lowerKickerReverseSpeed", lowerKickerReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, upperKickerRunSpeedSetter, "Control Variables", "upperKickerRunSpeed", upperKickerRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, upperKickerReverseSpeedSetter, "Control Variables", "upperKickerReverseSpeed", upperKickerReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, shooterRunSpeedSetter, "Control Variables", "shooterRunSpeed", shooterRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, shooterReverseSpeedSetter, "Control Variables", "shooterReverseSpeed", shooterReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, intakeRunSpeedSetter, "Control Variables", "intakeRunSpeed", intakeRunSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, intakeReverseSpeedSetter, "Control Variables", "intakeReverseSpeed", intakeReverseSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(m_driveSystem, maxDriveInputSetter, "Control Variables", "maxDriveInput", Constants.DriveConstants.INITIAL_MAX_INPUT);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, autoRotateSpeedSetter, "Control Variables", "autoRotateSpeed", autoRotateSpeed);
      RobotUtils.sendNumberSetterToShuffleboard(robotContainer, teleopShootSpeedSetter, "Control Variables", "teleopShootSpeed", teleopShootSpeed);
    } catch (NoSuchMethodException | SecurityException e) {
      SmartDashboard.putString("depressing_error", e.toString());
    }
  }

  public void sendMotorInversionsToShuffleboard() {
    try {
      RobotContainer robotContainer = this;

      Method setClimberRunInverted = ClimberSystem.class.getMethod("setClimberRunInverted", boolean.class);
      Method setClimberDeployInverted = ClimberSystem.class.getMethod("setClimberDeployInverted", boolean.class);
      Method setIntakeInverted = IntakeSystem.class.getMethod("setIntakeInverted", boolean.class);
      Method setShooterInverted = ShooterSystem.class.getMethod("setClimberRunInverted", boolean.class);
      Method setLowerKickerInverted = ShooterSystem.class.getMethod("setLowerKickerInverted", boolean.class);
      Method setUpperKickerInverted = ShooterSystem.class.getMethod("setUpperKickerInverted", boolean.class);

      RobotUtils.sendBooleanSetterToShuffleboard(climberSystem, setClimberRunInverted, "Motor Inversions", "setClimberRunInverted", Constants.ClimberConstants.CLIMBER_RUN_INVERSED);
      RobotUtils.sendBooleanSetterToShuffleboard(climberSystem, setClimberDeployInverted, "Motor Inversions", "setClimberDeployInverted", Constants.ClimberConstants.CLIMBER_DEPLOY_INVERSED);
      RobotUtils.sendBooleanSetterToShuffleboard(intakeSystem, setIntakeInverted, "Motor Inversions", "setIntakeInverted", Constants.IntakeConstants.INTAKE_RUN_INVERSED);
      RobotUtils.sendBooleanSetterToShuffleboard(shooterSystem, setShooterInverted, "Motor Inversions", "setShooterInverted", Constants.ShooterConstants.SHOOTER_INVERSED);
      RobotUtils.sendBooleanSetterToShuffleboard(shooterSystem, setLowerKickerInverted, "Motor Inversions", "setLowerKickerInverted", Constants.ShooterConstants.LOWER_KICKER_INVERSED);
      RobotUtils.sendBooleanSetterToShuffleboard(shooterSystem, setUpperKickerInverted, "Motor Inversions", "setUpperKickerInverted", Constants.ShooterConstants.UPPER_KICKER_INVERSED);
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
    SmartDashboard.putNumber("shooterDesiredRPM", teleopShootSpeed);
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
      .whileHeld(() -> shooterSystem.ShooterReverse(shooterReverseSpeed))
      .whenReleased(() -> shooterSystem.ShooterStop());
    //shooter (Y)
    new JoystickButton(controller, Button.kY.value)
      .whileHeld(() -> shooterSystem.ShooterRunVoltage(6))
      .whenReleased(() -> shooterSystem.ShooterStop());

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
    if(controller.getRightTriggerAxis() == 1)
      new TeleOpShoot(lowerKickerRunSpeed, upperKickerRunSpeed, shooterSystem).schedule();
    else if (oldRightTriggerAxis == 1) {
      shooterSystem.ShooterStop();
      shooterSystem.LowerKickerStop();
      shooterSystem.UpperKickerStop();

    }

    //d-pad
    //climber runner up (Up)
    if (controller.getPOV() == 0)
      climberSystem.ClimberRunnerUp(climberRunnerRunSpeed);
    //climber runner down (Down)
    else if (controller.getPOV() == 180)
      climberSystem.ClimberRunnerDown(climberRunnerReverseSpeed);
    //climber deployer up (Left)
    else if (controller.getPOV() == 270)
      climberSystem.ClimberDeployerUp(climberDeployerRunSpeed);
    //climber deployer down (Right)
    else if (controller.getPOV() == 90)
      climberSystem.ClimberDeployerDown(climberDeployerReverseSpeed);
    else if (controller.getPOV() < 0 && oldPOV >= 0) {
      climberSystem.ClimberStop();
      climberSystem.ClimberDeployerStop();

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
    //return new OneBallAuto(m_driveSystem, positioningSystem, shooterSystem);
    return new TwoBallAutoSimple(m_driveSystem, intakeSystem, positioningSystem, shooterSystem, startLocation, 0, Math.sqrt(Math.pow(ballPosition[0], 2) + Math.pow(ballPosition[1], 2)));
    //return new TwoBallAutoComplex(ballSystem, m_driveSystem, positioningSystem, intakeSystem, shooterSystem, almostBallPosition, almostStartPosition);
    
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

  public void setAutoRotateSpeed(Double speed) {
    autoRotateSpeed = speed;
  }

  public void setTeleopShootSpeed (Double speed) {
    teleopShootSpeed = speed;
  }
}
