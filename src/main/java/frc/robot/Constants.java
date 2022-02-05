package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_MOTOR_1_PORT = 1;
    public static final int LEFT_MOTOR_2_PORT = 2;
    public static final int RIGHT_MOTOR_1_PORT = 3;
    public static final int RIGHT_MOTOR_2_PORT = 4;
    public static final boolean RIGHT_MOTORS_INVERSED = true;
    public static final boolean LEFT_MOTORS_INVERSED = false;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_RUN_PORT = 5;
    public static final int INTAKE_DEPLOY_PORT = 6;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_RUN_PORT = 10;
    public static final int CLIMBER_DEPLOY_PORT = 11;
  }
}