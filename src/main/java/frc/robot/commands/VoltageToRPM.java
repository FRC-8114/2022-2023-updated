package frc.robot.commands;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class VoltageToRPM extends CommandBase{
    private ShooterSystem shooterSystem;
    private Timer timer;

    private List<Double> RPMs;

    private double voltage;

    private final double timeBeforeCalculation = 3;
    public VoltageToRPM(ShooterSystem shooterSystem, double voltage) {
        this.shooterSystem = shooterSystem;
        timer = new Timer();

        RPMs = new LinkedList<>();

        this.voltage = voltage;

    }
    public void initialize() {
        timer.start();

    }
    public void execute() {
        shooterSystem.ShooterRunVoltage(voltage);
        if (timer.get() > timeBeforeCalculation) {
            RPMs.add(shooterSystem.ShooterRPM);
            Object[] arrayRPMs = RPMs.toArray();
            Arrays.sort(arrayRPMs);

            SmartDashboard.putString("VoltageToRPM medianRPM", arrayRPMs[arrayRPMs.length / 2].toString());

        }
    
    }
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        shooterSystem.ShooterStop();

    }
    public boolean isFinished() {
        if (timer.get() >= 8)
            return true;
        return false;

    }
    
}
