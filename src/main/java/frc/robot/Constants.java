package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_MOTOR_1_PORT = 2;
    public static final int LEFT_MOTOR_2_PORT = 1;
    public static final int RIGHT_MOTOR_1_PORT = 3;
    public static final int RIGHT_MOTOR_2_PORT = 4;
    public static final boolean RIGHT_MOTORS_INVERSED = true;
    public static final boolean LEFT_MOTORS_INVERSED = false;

    public static final int[] kLeftEncoderPorts = new int[] {2, 1};
    public static final int[] kRightEncoderPorts = new int[] {3, 4};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
    public static final double kTrackwidthMeters = 1.214821901;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double ENCODER_DISTANCE_PER_PULSE = 1;

    public static final double INITIAL_MAX_VELOCITY = 0;
    public static final double INITIAL_CURVATURE_MAX_CURVATURE = 0;
    public static final double INITIAL_ARCADE_MAX_CURVATURE = 0;
    public static final double INITIAL_RAMP_RATE = 0;
        // Assumes the encoders are directly mounted on the wheel shafts
        //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double ksVolts = 0.087;
    public static final double kvVoltSecondsPerMeter = 2.91;
    public static final double kaVoltSecondsSquaredPerMeter = 0.303;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 11.4;
  }

  public static final class ShooterConstants {
    public static final double SHOOTER_DISTANCE_PER_PULSE = 0;
    public static final double VELOCITY_CONVERSION_FACTOR = 0;
    public static final double ENCODER_DISTANCE_PER_PULSE = 0;
    public static double MAX_INPUT = 0;
    public static final int LEFT_SHOOTER_CONTROLLER_PORT = 0;
    public static final int RIGHT_SHOOTER_CONTROLLER_PORT = 0;
  }
  public static final class IntakeConstants
  {
      public static final int INTAKE_CONTROLLER_PORT = 0;
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = .5;
    public static final double kMaxAccelerationMetersPerSecondSquared = .05;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}