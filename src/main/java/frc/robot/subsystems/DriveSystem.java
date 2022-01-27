package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSystem extends SubsystemBase {
  // The motors on the left side of the drive.
  public final static CANSparkMax leftMotorLeader = new CANSparkMax(2, MotorType.kBrushless);
  public final static CANSparkMax leftMotorFollower = new CANSparkMax(1, MotorType.kBrushless);

  // The motors on the right side of the drive.
  public final static CANSparkMax rightMotorLeader = new CANSparkMax(4, MotorType.kBrushless);
  public final static CANSparkMax rightMotorFollower = new CANSparkMax(3, MotorType.kBrushless);

  // The robot's drive
  private static final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

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

}