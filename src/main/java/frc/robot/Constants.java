package frc.robot;

public final class Constants {
  public static final double INCHES_TO_METERS = 0.0254;
  public static final double RPM_TO_VOLTAGE = 2.23*Math.pow(10,-3);
  public static final double RPM_TO_VOLTAGE_CONSTANT = .685;
  public static final double BALL_RADIUS = 4.75;

  public static final class DriveConstants {
    public static final int LEFT_MOTOR_1_PORT = 1;
    public static final int LEFT_MOTOR_2_PORT = 2;
    public static final int RIGHT_MOTOR_1_PORT = 3;
    public static final int RIGHT_MOTOR_2_PORT = 4;

    public static final boolean RIGHT_MOTORS_INVERSED = false;
    public static final boolean LEFT_MOTORS_INVERSED = true;
    public static final boolean STEERING_INVERSED = false;

    public static final double INITIAL_MAX_INPUT = .75;
  }

  public static final class ShooterConstants {
    public static final int UPPER_KICKER_CONTROLLER_PORT = 7;
    public static final int LOWER_KICKER_CONTROLLER_PORT = 8;
    public static final int SHOOTER_CONTROLLER_PORT = 9;

    public static final boolean SHOOTER_INVERSED = false;
    public static final boolean UPPER_KICKER_INVERSED = false;
    public static final boolean LOWER_KICKER_INVERSED = false;

    public static final double SHOOTER_DISTANCE_PER_PULSE = 1/42 * Math.PI * 5;
    public static final double VELOCITY_CONVERSION_FACTOR = 1/42 * Math.PI * 5;
    public static final double ENCODER_DISTANCE_PER_PULSE = 0;
    public static final double MAX_INPUT = 0;
	  public static final int SHOOTER_SPIN_UP_TIME = 3;

    public static final double TELEOP_DESIRED_VOLTAGE = 5.5;
    public static final double AUTO_DESIRED_VOLTAGE = 6.5;
    public static final double TELEOP_DESIRED_RPM = 3000;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_RUN_PORT = 5;

    public static final boolean INTAKE_RUN_INVERSED = false;

    public static final double INTAKE_LENGTH = 14;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_RUN_PORT = 10;
    public static final int CLIMBER_DEPLOY_PORT = 11;

    public static final boolean CLIMBER_RUN_INVERSED = false;
    public static final boolean CLIMBER_DEPLOY_INVERSED = true;
    public static final double CLIMBER_DEPLOY_CONVERSTION_FACTOR = 0;
  }

  public static final class PositioningConstants {
    public static final double GEAR_RATIO = 1 / 10.71;
    public static final double DRIVE_WHEEL_DIAMETER = 6;
    public static final double DRIVE_WHEEL_CIRCUMFRENCE = Math.PI * DRIVE_WHEEL_DIAMETER;

    public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO * DRIVE_WHEEL_CIRCUMFRENCE;

    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * INCHES_TO_METERS;
    public static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double DISTANCE_BETWEEN_WHEELS = 21.5;
    public static final double DISTANCE_BETWEEN_SPARK_MAXES = 8 + 3/16;
    // The comments describe what point is assigned to what letter on the whiteboard.
    public static final double[] SPAWN_ONE = new double[] {296.563, -139.023}; //A
    public static final double[] SPAWN_TWO = new double[] {300.661, -189.075}; //B
    public static final double[] SPAWN_THREE = new double[] {324.997, -198.840}; //C
    public static final double[] RED_SPAWN_ONE = new double[] {350.714, -184.977}; //D
    public static final double[] RED_SPAWN_TWO = new double[] {346.616, -134.925}; //E
    public static final double[] RED_SPAWN_THREE = new double[] {325.356, -126.340}; //F
    //Number is in order of what is closest to the spawns (ex: Ball One = Spawn one...)
    public static final double[] BALL_ONE = new double[] {194.242, -80.357};
    public static final double[] BALL_TWO = new double[] {198.693, -250.303};
    public static final double[] BALL_THREE = new double[] {297.728, -312.790};
    public static final double[] RED_BALL_ONE = new double[] {453.035, -243.643};
    public static final double[] RED_BALL_TWO = new double[] {448.585, -73.697};
    public static final double[] RED_BALL_THREE = new double[] {349.549, -11.210};
  }

  public static final class ControlConstants {
    public static final double LOWER_KICKER_INITIAL_RUN_SPEED = .5;
    public static final double LOWER_KICKER_INITIAL_REVERSE_SPEED = .75;
    public static final double UPPER_KICKER_INITIAL_RUN_SPEED = .35;
    public static final double UPPER_KICKER_INITIAL_REVERSE_SPEED = .35;
    public static final double SHOOTER_INITIAL_RUN_SPEED = .8;
    public static final double SHOOTER_INITIAL_REVERSE_SPEED = .4;
    public static final double INTAKE_INITIAL_RUN_SPEED = .6;
    public static final double INTAKE_INITIAL_REVERSE_SPEED = .25;
    public static final double CLIMBER_RUNNER_INITIAL_RUN_SPEED = .8;
    public static final double CLIMBER_RUNNER_INITIAL_REVERSE_SPEED = .8;
    public static final double CLIMBER_DEPLOYER_INITIAL_RUN_SPEED = .6;
    public static final double CLIMBER_DEPLOYER_INITIAL_REVERSE_SPEED = .6;
  }

  public static final class AutoConstants {
    public static final double AUTO_ROTATE_SPEED = 0.2;
  }
}