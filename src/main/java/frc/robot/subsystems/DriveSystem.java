package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSystem extends SubsystemBase {
  // The motors on the left side of the drive.
  public CANSparkMax leftMotorLeader = new CANSparkMax(DriveConstants.LEFT_MOTOR_1_PORT, MotorType.kBrushless);
  public CANSparkMax leftMotorFollower = new CANSparkMax(DriveConstants.LEFT_MOTOR_2_PORT, MotorType.kBrushless);

  // The motors on the right side of the drive.
  public CANSparkMax rightMotorLeader = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1_PORT, MotorType.kBrushless);
  public CANSparkMax rightMotorFollower = new CANSparkMax(DriveConstants.RIGHT_MOTOR_2_PORT, MotorType.kBrushless);

  // Creates and initializes the encoders
  public final RelativeEncoder leftLeaderEncoder = leftMotorLeader.getEncoder();
  public final RelativeEncoder leftFollowerEncoder = leftMotorFollower.getEncoder();
  public final RelativeEncoder rightLeaderEncoder = rightMotorLeader.getEncoder();
  public final RelativeEncoder rightFollowerEncoder = rightMotorFollower.getEncoder();

  // Creates a DifferentialDrive object
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  // Measures which side of the robot is considered front
  private boolean steeringInversed = Constants.DriveConstants.STEERING_INVERSED;

  //private DifferentialDriveOdometry m_odometry;
  private double curvatureMaxCurvature = 1.0;
  //private double arcadeMaxCurvature;
  //private double[] currentSpeeds = new double[2];

  public boolean back = false;
  public boolean isArcade = false;

  /** Creates a new DriveSubsystem. */
  public DriveSystem () {
    /* Initialize the drivetrain motors */

    configureDriveToDefault();
  }

  /**
   * Sets the drive motors to their default configuration
   */
  public void configureDriveToDefault() {
    // Left Leader Initialization
    leftMotorLeader.restoreFactoryDefaults();
    leftMotorLeader.setIdleMode(IdleMode.kCoast);
    leftMotorLeader.setInverted(DriveConstants.LEFT_MOTORS_INVERSED);
    leftMotorLeader.setOpenLoopRampRate(DriveConstants.INITIAL_RAMP_RATE);

    // Left Follower Initialization
    leftMotorFollower.restoreFactoryDefaults();
    leftMotorFollower.setIdleMode(IdleMode.kCoast);
    leftMotorFollower.setInverted(DriveConstants.LEFT_MOTORS_INVERSED);
    leftMotorFollower.follow(leftMotorLeader, false);
    leftMotorFollower.setOpenLoopRampRate(DriveConstants.INITIAL_RAMP_RATE);

    // Right Leader Initialization
    rightMotorLeader.restoreFactoryDefaults();
    rightMotorLeader.setIdleMode(IdleMode.kCoast);
    rightMotorLeader.setInverted(DriveConstants.RIGHT_MOTORS_INVERSED);
    rightMotorLeader.setOpenLoopRampRate(DriveConstants.INITIAL_RAMP_RATE);

    // Right Follower Initialization
    rightMotorFollower.restoreFactoryDefaults();
    rightMotorFollower.setIdleMode(IdleMode.kCoast);
    rightMotorFollower.setInverted(DriveConstants.RIGHT_MOTORS_INVERSED);
    rightMotorFollower.follow(rightMotorLeader, false);
    rightMotorFollower.setOpenLoopRampRate(DriveConstants.INITIAL_RAMP_RATE);

    setMaxInput(DriveConstants.INITIAL_MAX_INPUT);
  }

  // Switches which side of the robot is considered front
  public void switchMotorPorts () {
    steeringInversed = !steeringInversed;
    isArcade = !isArcade;
  }

  // Will return the average distance from the encoders on the left side of the drivetrain
  public double getLeftDistance () {
    return (leftLeaderEncoder.getPosition() + leftFollowerEncoder.getPosition()) / 2;
  }

  // Will return the average distance from the encoders on the right side of the drivetrain
  public double getRightDistance () {
    return (rightLeaderEncoder.getPosition() + rightFollowerEncoder.getPosition()) / 2;
  }

  // Will return the average distance from the left and right sides
  public double getTotalDistance () {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public void cheesyDrive (double speed, double curvature, boolean isArcade) {
    if (back) {
      curvature *= -1;
    }
    if (!isArcade) {
      // Applies a maximum curvature to curvature mode, limiting the minimum turn
      // radius
      m_drive.curvatureDrive(speed, -curvature * curvatureMaxCurvature, isArcade);
    } else {
      // m_drive.curvatureDrive(speed, curvature * arcadeMaxCurvature, isArcade);
      m_drive.arcadeDrive(speed, -curvature);
    }
  }

  /**
   * Drives the robot using tank drive controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void tankDrive (double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  // Drives the robot using arcade drive controls
  public void arcadeDrive (double speed, double rotation) {
    if(steeringInversed) {
      m_drive.arcadeDrive(-speed, -rotation);
    }
    
    else {
      m_drive.arcadeDrive(speed, -rotation);
    }
  }

  // Sets the maximum input accepted by the drivetrain to be a certain number
  public void setMaxInput (Double input) {
    m_drive.setMaxOutput(input);
  }

  // Switches the drivetrain between brake mode and coast mode 
  public void overwriteBrake (boolean brakeMode) {
    if (brakeMode) {
      leftMotorLeader.setIdleMode(IdleMode.kBrake);
      leftMotorFollower.setIdleMode(IdleMode.kBrake);
      rightMotorLeader.setIdleMode(IdleMode.kBrake);
      rightMotorFollower.setIdleMode(IdleMode.kBrake);
    }
    
    else {
      leftMotorLeader.setIdleMode(IdleMode.kCoast);
      leftMotorFollower.setIdleMode(IdleMode.kCoast);
      rightMotorLeader.setIdleMode(IdleMode.kCoast);
      rightMotorFollower.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Sets the ramp rate of the drive train motors
   * 
   * @param rampRate
   */
  public void setRampRate(double rampRate) {
    leftMotorLeader.setOpenLoopRampRate(rampRate);
    leftMotorFollower.setOpenLoopRampRate(rampRate);
    rightMotorLeader.setOpenLoopRampRate(rampRate);
    rightMotorFollower.setOpenLoopRampRate(rampRate);
  }
}