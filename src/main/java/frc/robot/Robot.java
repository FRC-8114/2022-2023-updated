package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Instantiates RobotContainer. Performs all button bindings, and puts
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.positioningSystem.updatePosition();
    
    updateShuffleboardValues();

    CommandScheduler.getInstance().run();
  }

  public void updateShuffleboardValues() {
    RobotUtils.sendNumberToShuffleboard("xPos", m_robotContainer.positioningSystem.position[0]);
    RobotUtils.sendNumberToShuffleboard("yPos", m_robotContainer.positioningSystem.position[1]);

    double averageOldLeftEncoders = (m_robotContainer.positioningSystem.oldLeftEncoderValues[0] + m_robotContainer.positioningSystem.oldLeftEncoderValues[1]) / 2;
    RobotUtils.sendNumberToShuffleboard("oldLeftEncoders", averageOldLeftEncoders);
    double averageOldRightEncoders = (m_robotContainer.positioningSystem.oldRightEncoderValues[0] + m_robotContainer.positioningSystem.oldRightEncoderValues[1]) / 2;
    RobotUtils.sendNumberToShuffleboard("oldRightEncoders", averageOldRightEncoders);

    RobotUtils.sendNumberToShuffleboard("leftLeaderEncoder", m_robotContainer.m_driveSystem.leftLeaderEncoder.getPosition());
    RobotUtils.sendNumberToShuffleboard("leftFollowerEncoder", m_robotContainer.m_driveSystem.leftFollowerEncoder.getPosition());
    double averageLeftEncoders = (m_robotContainer.m_driveSystem.leftLeaderEncoder.getPosition() + m_robotContainer.m_driveSystem.leftFollowerEncoder.getPosition()) / 2;
    RobotUtils.sendNumberToShuffleboard("leftEncoders", averageLeftEncoders);
    
    RobotUtils.sendNumberToShuffleboard("rightLeaderEncoder", m_robotContainer.m_driveSystem.rightLeaderEncoder.getPosition());
    RobotUtils.sendNumberToShuffleboard("rightFollowerEncoder", m_robotContainer.m_driveSystem.rightFollowerEncoder.getPosition());
    double averageRightEncoders = (m_robotContainer.m_driveSystem.rightLeaderEncoder.getPosition() + m_robotContainer.m_driveSystem.rightFollowerEncoder.getPosition()) / 2;
    RobotUtils.sendNumberToShuffleboard("rightEncoders", averageRightEncoders);

    double absoluteAverageLeftDifference = Math.abs(averageLeftEncoders - averageOldLeftEncoders);
    RobotUtils.sendNumberToShuffleboard("leftDifference", absoluteAverageLeftDifference);
    double absoluteAverageRightDifference = Math.abs(averageRightEncoders - averageOldRightEncoders);
    RobotUtils.sendNumberToShuffleboard("rightDifference", absoluteAverageRightDifference);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.m_driveSystem.overwriteBrake(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_driveSystem.overwriteBrake(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotContainer.m_driveSystem.cheesyDrive(m_robotContainer.controller.getLeftY(), m_robotContainer.controller.getRightX(), m_robotContainer.m_driveSystem.isArcade);
    m_robotContainer.m_driveSystem.arcadeDrive(m_robotContainer.controller.getLeftY(), m_robotContainer.controller.getRightX());
    m_robotContainer.periodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
