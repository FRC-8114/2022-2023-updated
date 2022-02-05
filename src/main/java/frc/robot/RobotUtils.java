package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotUtils {
    public static void sendNumberToShuffleboard(String name, double val) {
        SmartDashboard.putNumber(name, val);
    }  
}
