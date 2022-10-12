
package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class AdjustablePID {
  public SparkMaxPIDController pidController;
  public CANSparkMax motor;
  public RelativeEncoder encoder;
  public String pidName;
  public NetworkTableEntry processVariableEntry, outputEntry, actualP;

  public AdjustablePID(CANSparkMax motor, String pidName, double[] outputRange) {
    // initialize motor
    this.motor = motor;
    this.pidName = pidName;

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    pidController = motor.getPIDController();
    encoder = motor.getEncoder();

    // set PID coefficients
    pidController.setP(5e-5);
    pidController.setI(1e-6);
    pidController.setD(0);
    pidController.setIZone(0);
    pidController.setFF(0.000156);
    pidController.setOutputRange(outputRange[0], outputRange[1]);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    RobotUtils.sendToShuffleboard(pidName + " P Gain", 5e-5);
    RobotUtils.sendToShuffleboard(pidName + " I Gain", 1e-6);
    RobotUtils.sendToShuffleboard(pidName + " D Gain", 0);
    RobotUtils.sendToShuffleboard(pidName + " I Zone", 0);
    RobotUtils.sendToShuffleboard(pidName + " Feed Forward", 0.000156);
    RobotUtils.sendToShuffleboard(pidName + " Max Output", outputRange[0]);
    RobotUtils.sendToShuffleboard(pidName + " Min Output", outputRange[1]);

    // display Smart Motion coefficients
    RobotUtils.sendToShuffleboard(pidName + " Max Velocity", 2000);
    RobotUtils.sendToShuffleboard(pidName + " Min Velocity", 0);
    RobotUtils.sendToShuffleboard(pidName + " Max Acceleration", 1500);
    RobotUtils.sendToShuffleboard(pidName + " Allowed Closed Loop Error", 0);
    RobotUtils.sendToShuffleboard(pidName + " Set Position", 0);
    RobotUtils.sendToShuffleboard(pidName + " Set Velocity", 0);
    RobotUtils.sendToShuffleboard(pidName + " SetPoint", 0);

    // button to toggle between velocity and smart motion modes
    RobotUtils.sendToShuffleboard(pidName + " Mode", true);
  }

  public AdjustablePID(CANSparkMax motor, String pidName, double kP, double kI, double kD, double kIz, double kFF, double[] outputRange) {
    // initialize motor
    this.motor = motor;
    this.pidName = pidName;

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    pidController = motor.getPIDController();
    encoder = motor.getEncoder();

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(outputRange[0], outputRange[1]);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    RobotUtils.sendToShuffleboard(pidName + " P Gain", kP);
    RobotUtils.sendToShuffleboard(pidName + " I Gain", kI);
    RobotUtils.sendToShuffleboard(pidName + " D Gain", kD);
    RobotUtils.sendToShuffleboard(pidName + " I Zone", kIz);
    RobotUtils.sendToShuffleboard(pidName + " Feed Forward", kFF);
    RobotUtils.sendToShuffleboard(pidName + " Max Output", outputRange[0]);
    RobotUtils.sendToShuffleboard(pidName + " Min Output", outputRange[1]);

    // display Smart Motion coefficients
    RobotUtils.sendToShuffleboard(pidName + " Max Velocity", 2000);
    RobotUtils.sendToShuffleboard(pidName + " Min Velocity", 0);
    RobotUtils.sendToShuffleboard(pidName + " Max Acceleration", 1500);
    RobotUtils.sendToShuffleboard(pidName + " Allowed Closed Loop Error", 0);
    RobotUtils.sendToShuffleboard(pidName + " Set Position", 0);
    RobotUtils.sendToShuffleboard(pidName + " Set Velocity", 0);
    RobotUtils.sendToShuffleboard(pidName + " SetPoint", 0);

    // button to toggle between velocity and smart motion modes
    RobotUtils.sendToShuffleboard(pidName + " Mode", true);
  }

  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = RobotUtils.retrieveFromShuffleboard(pidName + " P Gain", 0);
    double i = RobotUtils.retrieveFromShuffleboard(pidName + " I Gain", 0);
    double d = RobotUtils.retrieveFromShuffleboard(pidName + " D Gain", 0);
    double iz = RobotUtils.retrieveFromShuffleboard(pidName + " I Zone", 0);
    double ff = RobotUtils.retrieveFromShuffleboard(pidName + " Feed Forward", 0);
    double max = RobotUtils.retrieveFromShuffleboard(pidName + " Max Output", 0);
    double min = RobotUtils.retrieveFromShuffleboard(pidName + " Min Output", 0);
    double maxV = RobotUtils.retrieveFromShuffleboard(pidName + " Max Velocity", 0);
    double minV = RobotUtils.retrieveFromShuffleboard(pidName + " Min Velocity", 0);
    double maxA = RobotUtils.retrieveFromShuffleboard(pidName + " Max Acceleration", 0);
    double allE = RobotUtils.retrieveFromShuffleboard(pidName + " Allowed Closed Loop Error", 0);

    double kP = pidController.getP();
    double kI = pidController.getI();
    double kD = pidController.getD();
    double kIz = pidController.getIZone();
    double kFF = pidController.getFF();
    double kMaxOutput = pidController.getOutputMin();
    double kMinOutput = pidController.getOutputMax();
    double maxVel = pidController.getSmartMotionMaxVelocity(0);
    double minVel = pidController.getSmartMotionMinOutputVelocity(0);
    double maxAcc = pidController.getSmartMotionMaxAccel(0);
    double allowedErr = pidController.getSmartMotionAllowedClosedLoopError(0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) pidController.setP(p);
    if((i != kI)) pidController.setI(i); 
    if((d != kD)) pidController.setD(d);
    if((iz != kIz)) pidController.setIZone(iz);
    if((ff != kFF)) pidController.setFF(ff);
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
    }
    if((maxV != maxVel)) pidController.setSmartMotionMaxVelocity(maxV,0);
    if((minV != minVel)) pidController.setSmartMotionMinOutputVelocity(minV,0);
    if((maxA != maxAcc)) pidController.setSmartMotionMaxAccel(maxA,0);
    if((allE != allowedErr)) pidController.setSmartMotionAllowedClosedLoopError(allE,0);

    double setPoint, processVariable;
    boolean mode = RobotUtils.retrieveFromShuffleboard(pidName + " Mode", false);
    if(mode) {
      setPoint = RobotUtils.retrieveFromShuffleboard(pidName + " Set Velocity", 0);
      pidController.setReference(setPoint, ControlType.kVelocity);
      processVariable = encoder.getVelocity();
    } else {
      setPoint = RobotUtils.retrieveFromShuffleboard(pidName + " Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      pidController.setReference(setPoint, ControlType.kSmartMotion);
      processVariable = encoder.getPosition();
    }
    
    RobotUtils.sendToShuffleboard(pidName + " SetPoint", setPoint);
    RobotUtils.sendToShuffleboard(pidName + " Process Variable", processVariable);
    RobotUtils.sendToShuffleboard(pidName + " Output", motor.getAppliedOutput());

    RobotUtils.sendToShuffleboard(pidName + " PID P", pidController.getP());
    RobotUtils.sendToShuffleboard(pidName + " PID I", pidController.getI());
    RobotUtils.sendToShuffleboard(pidName + " PID D", pidController.getD());
    RobotUtils.sendToShuffleboard(pidName + " PID Iz", pidController.getIZone());
    RobotUtils.sendToShuffleboard(pidName + " PID FF", pidController.getFF());
  }
}