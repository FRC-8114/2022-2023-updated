public static class RobotUtils {
    public static void sendNumberToShuffleboard(String name, Object val) {
        SmartDashboard.putNumber(name, val);
    }  
}
