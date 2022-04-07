package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTrackingSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.FieldPositioningSystem;

public class RotateToBall extends SequentialCommandGroup {
    // Calculates the angle that the bot should rotate to, then rotates to said angle
    public RotateToBall(BallTrackingSystem ballSystem, FieldPositioningSystem positioningSystem, DriveSystem driveSystem) {
        // Begin with the current angle
        double angle = positioningSystem.navxAngle;

        ArrayList<double[]> balls = ballSystem.entitiesDataFromJSon(ballSystem.ballInfo.getString(""));
        double[] bestBall = ballSystem.mostConfidentEntity(balls);
        double[] bestBallPos = ballSystem.entityCenter(bestBall);
        // Gets the angle form the left side of the camera feed to the detected cargo
        double bestBallAngle = ballSystem.pixelPosToAngle(bestBallPos[0]);

        // Adjusts current angle by the angle to the ball (mildly off as the camera isn't centered)
        angle += bestBallAngle;

        addCommands(new RotateToAngle(driveSystem, positioningSystem, angle, 0.2));
    }
}
