package frc.robot;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotUtils {
    /**
     * Sends the given value to shuffleboard via SmartDashboard using the given
     * identifier
     * 
     * @param identifier      identifier
     * @param val       the value to send
     */
    // For numbers
    public static void sendToShuffleboard(String identifier, double val) {
        SmartDashboard.putNumber(identifier, val);
    }
    // For strings
    public static void sendToShuffleboard(String identifier, String val) {
        SmartDashboard.putString(identifier, val);
    }
    // For booleans
    public static void sendToShuffleboard(String identifier, boolean val) {
        SmartDashboard.putBoolean(identifier, val);
    }
    // For number arrays
    public static void sendToShuffleboard(String identifier, double[] val) {
        SmartDashboard.putNumberArray(identifier, val);
    }
    // For string arrays
    public static void sendToShuffleboard(String identifier, String[] val) {
        SmartDashboard.putStringArray(identifier, val);
    }
    public static void sendToShuffleboard(String identifier, boolean[] val) {
        SmartDashboard.putBooleanArray(identifier, val);
    }

    /**
     * Retrieves the given value from shuffleboard via SmartDashboard using the given
     * identifier and the given default
     * 
     * @param identifier      identifier
     * @param val             the default value if the given identifier returns a null
     */
    // For numbers
    public static double retrieveFromShuffleboard(String identifier, double defaultVal) {
        return SmartDashboard.getNumber(identifier, defaultVal);
    }
    // For strings
    public static String retrieveFromShuffleboard(String identifier, String defaultVal) {
        return SmartDashboard.getString(identifier, defaultVal);
    }
    // For booleans
    public static boolean retrieveFromShuffleboard(String identifier, boolean defaultVal) {
        return SmartDashboard.getBoolean(identifier, defaultVal);
    }
    // For number arrays
    public static double[] retrieveFromShuffleboard(String identifier, double[] defaultVal) {
        return SmartDashboard.getNumberArray(identifier, defaultVal);
    }
    // For string arrays
    public static String[] retrieveFromShuffleboard(String identifier, String[] defaultVal) {
        return SmartDashboard.getStringArray(identifier, defaultVal);
    }
    public static boolean[] retrieveFromShuffleboard(String identifier, boolean[] defaultVal) {
        return SmartDashboard.getBooleanArray(identifier, defaultVal);
    }

    /**
     * Sends the given value to shuffleboard and assigns a networkTableEntry listener
     * to run the given setter whenever the value is updated
     * 
     * @param object            the class of which the setter is a method
     * @param setter            the setter to call when the value is altered
     * @param tab               the shuffleboard tab on which to display the value
     * @param identifier        the identifier to display the given value under
     * @param initialValue      the initial value to displayx
     */
    // For numbers
    public static void sendSetterToShuffleboard(Object object, Method setter, String tab, String identifier, double initialValue) {
        Shuffleboard.getTab(tab).add(identifier, initialValue).withWidget(BuiltInWidgets.kTextView).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getDouble());
        } catch (IllegalAccessException e) {
            SmartDashboard.putString("Illegal Access", e.toString());
        } catch (IllegalArgumentException e) {
            SmartDashboard.putString("Illegal Argument", e.toString());
        } catch (InvocationTargetException e) {
            SmartDashboard.putString("Target Exception", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
    // For strings
    public static void sendSetterToShuffleboard(Object object, Method setter, String tab, String identifier, String initialValue) {
        Shuffleboard.getTab(tab).add(identifier, initialValue).withWidget(BuiltInWidgets.kTextView).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getString());
        } catch (IllegalAccessException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            sendToShuffleboard("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
    // For booleans
    public static void sendSetterToShuffleboard(Object object, Method setter, String tab, String identifier, boolean initialValue) {
        Shuffleboard.getTab(tab).add(identifier, initialValue).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getBoolean());
        } catch (IllegalAccessException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            sendToShuffleboard("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
    // For number arrays
    public static void sendSetterToShuffleboard(Object object, Method setter, String tab, String identifier, double[] initialValue) {
        Shuffleboard.getTab(tab).add(identifier, initialValue).withWidget(BuiltInWidgets.kTextView).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getDouble());
        } catch (IllegalAccessException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            sendToShuffleboard("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
    // For string arrays
    public static void sendSetterToShuffleboard(Object object, Method setter, String tab, String identifier, String[] initialValue) {
        Shuffleboard.getTab(tab).add(identifier, initialValue).withWidget(BuiltInWidgets.kTextView).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getString());
        } catch (IllegalAccessException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            sendToShuffleboard("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
    // For boolean arrays
    public static void sendSetterToShuffleboard(Object object, Method setter, String tab, String identifier, boolean[] initialValue) {
        Shuffleboard.getTab(tab).add(identifier, initialValue).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        try {
            setter.invoke(object, event.value.getBoolean());
        } catch (IllegalAccessException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (IllegalArgumentException e) {
            sendToShuffleboard("depressing_error", e.toString());
        } catch (InvocationTargetException e) {
            sendToShuffleboard("depressing_error", e.toString());
        }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
}