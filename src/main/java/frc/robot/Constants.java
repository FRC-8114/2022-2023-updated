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

  public static final class ShooterConstants {
    public static final double SHOOTER_DISTANCE_PER_PULSE = 0;
    public static final double VELOCITY_CONVERSION_FACTOR = 0;
    public static final double ENCODER_DISTANCE_PER_PULSE = 0;
    public static double MAX_INPUT = 0;
    public static final int LEFT_SHOOTER_CONTROLLER_PORT = 0;
    public static final int RIGHT_SHOOTER_CONTROLLER_PORT = 0;
    public static final int KICKER_CONTROLLER_PORT = 0;
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