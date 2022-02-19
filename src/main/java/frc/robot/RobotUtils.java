package frc.robot;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotUtils {
    public static void sendNumberToShuffleboard(String name, double val) {
        SmartDashboard.putNumber(name, val);
    }  

    public static void sendNumberSetterToShuffleboard(Object object, Method setter, String tab, String name, double initialValue) {
    
        Shuffleboard.getTab(tab).add(name, initialValue).withWidget(BuiltInWidgets.kTextView).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getDouble());
            sendNumberToShuffleboard("work?", 1);
            sendNumberToShuffleboard("oldNumber", initialValue);
            sendNumberToShuffleboard("newNumber", event.value.getDouble());
        } catch (IllegalAccessException e) {
            SmartDashboard.putString("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            SmartDashboard.putString("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            SmartDashboard.putString("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        sendNumberToShuffleboard("work?", 0);
    }

    public static void sendBooleanSetterToShuffleboard(Object object, Method setter, String tab, String name, boolean initialValue) {
    
        Shuffleboard.getTab(tab).add(name, initialValue).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getBoolean());
        } catch (IllegalAccessException e) {
            SmartDashboard.putString("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            SmartDashboard.putString("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            SmartDashboard.putString("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        sendNumberToShuffleboard("work?", 0);
    }
}
