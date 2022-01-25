package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;

public class DriveSystem extends SubsystemBase {
  // The motors on the left side of the drive.
  public final static CANSparkMax leftMotorLeader = new CANSparkMax(DriveConstants.LEFT_MOTOR_1_PORT, MotorType.kBrushless);
  public final static CANSparkMax leftMotorFollower = new CANSparkMax(DriveConstants.LEFT_MOTOR_2_PORT, MotorType.kBrushless);

  // The motors on the right side of the drive.
  public final static CANSparkMax rightMotorLeader = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1_PORT, MotorType.kBrushless);
  public final static CANSparkMax rightMotorFollower = new CANSparkMax(DriveConstants.RIGHT_MOTOR_2_PORT, MotorType.kBrushless);

  // The robot's drive
  private static final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  // Odometry class for tracking robot position
  private static DifferentialDriveOdometry m_odometry;

  // Drivetrain encoders for position tracking
  static final RelativeEncoder leftLeaderEncoder = leftMotorLeader.getEncoder();
  static final RelativeEncoder rightLeaderEncoder = rightMotorLeader.getEncoder();

  // Control constants that govern the robot's movement 
  private double maxVelocity;
  private static double curvatureMaxCurvature;
  private static double arcadeMaxCurvature;
  private double[] currentSpeeds = new double[2];

  // NetworkTable Entries for debugging mimicking
  private static NetworkTableEntry speedEntry;
  private static NetworkTableEntry curvatureEntry;
  private static NetworkTableEntry isArcadeEntry;

  public static boolean back = false, driverControl = true, canAutoCenter = true;

  /** Creates a new DriveSubsystem. */
  public DriveSystem() {
    speedEntry = NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Drive_Speed");
    curvatureEntry = NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Drive_Quick_Turn");
    isArcadeEntry = NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Drive_Arcade?");

    /* Initialize the drivetrain motors */

    // Left Leader Initialization
    leftMotorLeader.restoreFactoryDefaults();
    leftMotorLeader.setIdleMode(IdleMode.kCoast);
    leftMotorLeader.setInverted(DriveConstants.LEFT_MOTORS_INVERSED);

    // Left Follower Initialization
    leftMotorFollower.restoreFactoryDefaults();
    leftMotorFollower.setIdleMode(IdleMode.kCoast);
    leftMotorFollower.setInverted(DriveConstants.LEFT_MOTORS_INVERSED);
    leftMotorFollower.follow(leftMotorLeader, false);

    // Right Leader Initialization
    rightMotorLeader.restoreFactoryDefaults();
    rightMotorLeader.setIdleMode(IdleMode.kCoast);
    rightMotorLeader.setInverted(DriveConstants.RIGHT_MOTORS_INVERSED);

    // Right Follower Initialization
    rightMotorFollower.restoreFactoryDefaults();
    rightMotorFollower.setIdleMode(IdleMode.kCoast);
    rightMotorFollower.setInverted(DriveConstants.RIGHT_MOTORS_INVERSED);
    rightMotorFollower.follow(rightMotorLeader, false);

    // Sets the distance per pulse for the encoders
    leftLeaderEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    rightLeaderEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);

    resetEncoders();

    // Initializes control constants
    maxVelocity = DriveConstants.INITIAL_MAX_VELOCITY;
    curvatureMaxCurvature = DriveConstants.INITIAL_CURVATURE_MAX_CURVATURE;
    arcadeMaxCurvature = DriveConstants.INITIAL_ARCADE_MAX_CURVATURE;
    setMaxOutput();
    setRampRate(DriveConstants.INITIAL_RAMP_RATE);

  }

  @Override
  public void periodic() {}

  public void emergencyStop(double time) {
    double startLeft = -currentSpeeds[0] / 5, startRight = -currentSpeeds[1] / 5;

    tankDrive(0, 0);
    Timer timer = new Timer();
    timer.start();
    timer.delay(.15);

    for (; timer.get() < time;) {
      tankDrive(startLeft, startRight);
    }

    tankDrive(0, 0);
    timer.stop();
  }

  /**
   * Drives the robot using cheesy drive controls.
   * 
   * Cheesy drive allows the driver to swap between arcade drive and curvature drive mid-match
   * this allows for the benefits of both.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public static void cheesyDrive(double speed, double curvature, boolean isArcade) {
    if (back) {
      speed = -speed;
      curvature = -curvature;
    }
    if (!isArcade) {
      // Applies a maximum curvature to curvature mode, limiting the minimum turn
      // radius
      m_drive.curvatureDrive(speed, curvature * curvatureMaxCurvature, isArcade);
    } else {
      m_drive.curvatureDrive(speed, curvature * arcadeMaxCurvature, isArcade);
    }

    speedEntry.forceSetDouble(speed);
    curvatureEntry.forceSetDouble(curvature);
    isArcadeEntry.forceSetBoolean(isArcade);
  }

  /**
   * Drives the robot using tank drive controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorLeader.setVoltage(leftVolts);
    rightMotorLeader.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Sets the max output of the drive to the value of maxOutput;
   */
  public void setMaxOutput() {
    m_drive.setMaxOutput(maxVelocity);
  }

  public void setRampRate(double rampRate) {
    rightMotorLeader.setOpenLoopRampRate(rampRate);
    rightMotorFollower.setOpenLoopRampRate(rampRate);
    leftMotorLeader.setOpenLoopRampRate(rampRate);
    leftMotorFollower.setOpenLoopRampRate(rampRate);
  }

  /**
   * Increments maxOutput by 0.05 and updates the differential drive's max output,
   * maxing at 1.0.
   * 
   * @param maxVelocity
   */
  public void incMaxSpeed() {
    maxVelocity = Math.min(maxVelocity + 0.05, 1.0);
    setMaxOutput();
  }

  /**
   * Decreases maxOutput by 0.05 and updates the differential drive's max output,
   * stopping at 0.1.
   * 
   * @param maxVelocity
   */
  public void decMaxSpeed() {
    maxVelocity = Math.max(maxVelocity - 0.05, 0.1);
    setMaxOutput();
  }

  /** Zeroes the heading of the robot. */
  public static void zeroHeading() {}

  /** Resets the drive encoders to currently read a position of 0. */
  public static void resetEncoders() {
    leftLeaderEncoder.setPosition(0);
    rightLeaderEncoder.setPosition(0);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  /**
   * Sets the value of the right motor's inverted boolean to the input value
   */
  public void setRightMotorsInverted(boolean inverted) {
    rightMotorLeader.setInverted(inverted);
    rightMotorFollower.setInverted(inverted);
  }

  /**
   * Sets the value of the right motor's inverted boolean to the input value
   */
  public void setLeftMotorsInverted(boolean inverted) {
    leftMotorLeader.setInverted(inverted);
    leftMotorFollower.setInverted(inverted);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeaderEncoder.getVelocity(), rightLeaderEncoder.getVelocity());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  /**public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }*/

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public static double getAverageEncoderDistance() {
    return (-1 * leftLeaderEncoder.getPosition() + rightLeaderEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the average velocity of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderVelocity() {
    return (leftLeaderEncoder.getVelocity() + rightLeaderEncoder.getVelocity()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return leftLeaderEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return rightLeaderEncoder;
  }

  public static void reverseDirection() {
    back = !back;
  }
}