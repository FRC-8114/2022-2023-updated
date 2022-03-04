package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSystem extends SubsystemBase {
  // The motors on the left side of the drive.
  public static CANSparkMax leftMotorLeader = new CANSparkMax(DriveConstants.LEFT_MOTOR_1_PORT, MotorType.kBrushless);
  public static CANSparkMax leftMotorFollower = new CANSparkMax(DriveConstants.LEFT_MOTOR_2_PORT, MotorType.kBrushless);

  // The motors on the right side of the drive.
  public static CANSparkMax rightMotorLeader = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1_PORT, MotorType.kBrushless);
  public static CANSparkMax rightMotorFollower = new CANSparkMax(DriveConstants.RIGHT_MOTOR_2_PORT, MotorType.kBrushless);

  // Creates and initializes the encoders
  public final static RelativeEncoder leftLeaderEncoder = leftMotorLeader.getEncoder();
  public final static RelativeEncoder leftFollowerEncoder = leftMotorFollower.getEncoder();
  public final static RelativeEncoder rightLeaderEncoder = rightMotorLeader.getEncoder();
  public final static RelativeEncoder rightFollowerEncoder = rightMotorFollower.getEncoder();

  // Creates a DifferentialDrive object
  private static final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  // Measures which side of the robot is considered front
  private boolean steeringInversed = Constants.DriveConstants.STEERING_INVERSED;

  private static DifferentialDriveOdometry m_odometry;
  private static double curvatureMaxCurvature = 1.0;
  private static double arcadeMaxCurvature;
  private double[] currentSpeeds = new double[2];

  public static boolean back = false;
  public static boolean isArcade = false;

  /** Creates a new DriveSubsystem. */
  public DriveSystem () {
  
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
  }

  // Switches which side of the robot is considered front
  public void switchMotorPorts () {
    steeringInversed = !steeringInversed;
  }

  // For use in cheesy drive, will switch back and forth from curvature drive to arcade drive
  public void switchDriveSystem () {
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

  public static void cheesyDrive (double speed, double curvature, boolean isArcade) {
    if (back) {
      curvature = curvature;
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
}