package frc.robot;


import java.lang.management.ManagementFactory;
import java.lang.management.RuntimeMXBean;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
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
    RuntimeMXBean mxbean = ManagementFactory.getRuntimeMXBean();
    long runtimeMilliseconds = mxbean.getUptime();
    RobotUtils.sendToShuffleboard("robotUptime", runtimeMilliseconds / 1000.0);

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_driveSystem.overwriteBrake(false);
  }

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
    m_robotContainer.m_driveSystem.overwriteBrake(false);
    m_robotContainer.m_driveSystem.configureDriveToDefault();
    // This makes sure that the autonomous stops running when
    // teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotContainer.m_driveSystem.cheesyDrive(m_robotContainer.controller.getLeftY(), m_robotContainer.controller.getRightX(), m_robotContainer.m_driveSystem.isArcade);
    m_robotContainer.getDriveSystem().arcadeDrive(m_robotContainer.getXboxController().getLeftY(), m_robotContainer.getXboxController().getRightX());
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
