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

  // The encoders
  public final static RelativeEncoder leftLeaderEncoder = leftMotorLeader.getEncoder();
  public final static RelativeEncoder leftFollowerEncoder = leftMotorFollower.getEncoder();
  public final static RelativeEncoder rightLeaderEncoder = rightMotorLeader.getEncoder();
  public final static RelativeEncoder rightFollowerEncoder = rightMotorFollower.getEncoder();

  // The robot's drive
  private static final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  public boolean steeringInversed = false;

  private static DifferentialDriveOdometry m_odometry;
  private static double curvatureMaxCurvature = 1.0;
  private static double arcadeMaxCurvature;
  private double[] currentSpeeds = new double[2];

  /** Creates a new DriveSubsystem. */
  public DriveSystem() {
  
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

    steeringInversed = Constants.DriveConstants.STEERING_INVERSED;
  }

  public static boolean back = false;
  public static boolean isArcade = false;

  public RelativeEncoder getEncoder(String encoderName) {
    switch (encoderName) {
      case "leftLeaderEncoder": return leftLeaderEncoder;
      case "leftFollowerEncoder": return leftFollowerEncoder;
      case "rightLeaderEncoder": return rightLeaderEncoder;
      case "rightFollowerEncoder": return rightFollowerEncoder;
      default: return null;
    }
  }

  public void invertMotors(boolean reversed) {
    rightMotorLeader.setInverted(reversed);
    leftMotorLeader.setInverted(reversed);
    back = reversed;
  }

  public void switchMotorPorts () {
    steeringInversed = !steeringInversed;
  }

  public void switchDriveSystem () {
    isArcade = !isArcade;
  }

  public double getLeftDistance() {
    return (leftLeaderEncoder.getPosition() + leftFollowerEncoder.getPosition()) / 2;
  }

  public double getRightDistance() {
    return (rightLeaderEncoder.getPosition() + rightFollowerEncoder.getPosition()) / 2;
  }

  public double getTotalDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public static void cheesyDrive(double speed, double curvature, boolean isArcade) {
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
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    if(steeringInversed) {
      m_drive.arcadeDrive(speed, -rotation);
    } else {
      m_drive.arcadeDrive(-speed, -rotation);
    }
    
  }

  public void setMaxInput(Double input) {
    m_drive.setMaxOutput(input);
  }
}