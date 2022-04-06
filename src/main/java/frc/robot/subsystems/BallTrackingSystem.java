package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotUtils;

public class BallTrackingSystem extends SubsystemBase {
    public NetworkTableEntry ballInfo;
    public double diagonalCameraFOV, diagonalCameraResolution;
    
    public BallTrackingSystem(double diagonalCameraFOV, double diagonalCameraResolution) {
        this.diagonalCameraFOV = diagonalCameraFOV;
        this.diagonalCameraResolution = diagonalCameraResolution;
    
        ballInfo = NetworkTableInstance.getDefault().getTable("ML").getEntry("detections");
    }

    /**
     * Periodically outputs the details of the best ball for debugging
     */
    public void periodic() {
        // Grabs the latest detection data
        String axonJson = ballInfo.getString("[]");

        // Don't attempt to operate on a nonexistend json
        if(axonJson.equals("[]")) {
            return;
        }

        // Parses the detection data
        ArrayList<String> entities = entitiesFromJson(axonJson);
        ArrayList<double[]> entitiesData = entitiesDataFromJSon(axonJson);

        // Important information regarding the best ball
        double[] bestBall = mostConfidentEntity(entitiesData);
        double[] bestBallPos = entityCenter(bestBall);
        double bestBallAngle = pixelPosToAngle(bestBallPos[0]);

        // Send debugging data to Shuffleboard
        RobotUtils.sendToShuffleboard("rawBallJson(s)", axonJson);
        
        RobotUtils.sendToShuffleboard("ballJson(s)", entities.toString());
        RobotUtils.sendToShuffleboard("bestBallPos", "x = "+ bestBallPos[0] +" | y = "+ bestBallPos[1]);
        RobotUtils.sendToShuffleboard("bestBallConfidence", bestBall[4]);
        RobotUtils.sendToShuffleboard("bestBallAngle", bestBallAngle);
    }

    /**
     * Returns the centerpoint of the given entity (may lie between pixels)
     * 
     * @param entity
     * @return          the pixel at the entities center
     */
    public double[] entityCenter(double[] entity) {
        //                       x     +  length  ,     y     +  height
        return new double[] {entity[0] + entity[2], entity[1] + entity[3]};
    }

    /**
     * Converts a pixel to an angle from the center of the
     * camera, this will work for both diagonal and vertical
     * positions
     * 
     * @param pixelPos  the pixel to get the angle of
     * @return          the angle of the vector from the camera center to the given pixel
     */
    public double pixelPosToAngle(double pixelPos) {
        return pixelPos * (diagonalCameraFOV/diagonalCameraResolution);
    }

    /**
     * Returns the most confident of the given entities (probably most likely to actually be a cargo)
     * 
     * @param entities
     * @return
     */
    public double[] mostConfidentEntity(ArrayList<double[]> entities) {
        double[] mostConfident = new double[] {0,0,0,0,0};

        // Replaces the current mostConfident if more confident
        for(double[] entity : entities) {
            if(entity[4] > mostConfident[4]) {
                mostConfident = entity;
            }
        }

        return mostConfident;
    }

    /**
     * Filters out entities that fall below the given confidence threshold
     * 
     * @param entities
     * @return
     */
    public ArrayList<double[]> onlyConfidentEntities(ArrayList<double[]> entities, double confidenceThreshold) {
        ArrayList<double[]> output = new ArrayList<double[]>();

        for(double[] entity : entities) {
            if(entity[4] >= confidenceThreshold) {
                output.add(entity);
            }
        }

        return output;
    }

    /**
     * Returns an ArrayList containing the data defining all detected cargo
     * 
     * @param axonJson  the raw json String from axon
     * @return          the data defining the detected cargo
     */
    public ArrayList<double[]> entitiesDataFromJSon(String axonJson) {
        ArrayList<double[]> output = new ArrayList<double[]>();
        
        // Splits the given json into detected entities
        ArrayList<String> entities = entitiesFromJson(axonJson);

        // Pulls the desired data from each entity
        for(String entity : entities) {
            output.add(entityDataFromJson(entity));
        }

        return output;
    }

    /**
     * A decently jank way to seperate the entities present in an
     * axon json
     * 
     * @param entityJson
     * @return              a string array of entity jsons
     */
    public ArrayList<String> entitiesFromJson(String axonJson) {
        ArrayList<String> output = new ArrayList<String>();
        ArrayList<String> temp = new ArrayList<String>(); // used to store potential entities before entity status is determined

        /**
         * The method below is incredibly stupid, but it does work
         * 
         * All json entities sent by follow the format:
         * {"label": *label*, "box": {"ymin": *ymin*, "xmin": *xmin*, "ymax": *ymax*, "xmax": *xmax*}, "confidence": *confidence*}
         * 
         * All entities in the same json are seperated by commas & whitespace
         * entity(ies) are always surrounded by square brackets
         * 
         * This means that an entity is 13 whitespace gaps long
         **/
        Scanner jsonScanner = new Scanner(axonJson);
        // Due to the nature of this loop
        while(jsonScanner.hasNext()) {
            // Check if you have already parsed a full entity
            if(temp.size() == 13) {
                // Reconstruct entity with original newlines
                String entity = "";

                for(String entityPart : temp) {
                    entity += entityPart +" ";
                }

                output.add(entity);
                // Reset temp
                temp.clear();
            }

            // Read the next line of the json
            temp.add(jsonScanner.next());
        }

        // Closes the scanner utilized
        jsonScanner.close();

        return output;
    }

    /**
     * An incredibly jank way to parse the entity jsons given by axon
     * 
     * @param entityJson
     * @return              the parsed data
     */
    public double[] entityDataFromJson(String entityJson) {
        // The following array will be in the form {x, y, length, height, confidence}
        double[] entityData = new double[5];
    
        /**
         * The method below is incredibly stupid, but it does work
         * 
         * All json entities sent by follow the format:
         * {"label": *label*, "box": {"ymin": *ymin*, "xmin": *xmin*, "ymax": *ymax*, "xmax": *xmax*}, "confidence": *confidence*}
         * 
         * thus breaking the string on whitespace yeilds the sequence:
         * {"label":
         * *label*,
         * "box":
         * {
         * "ymin":
         * *ymin*,
         * "xmin":
         * *xmin,
         * ...
         * 
         * Thus the terms of interest are at indicies 1, 4, 6, 8, 10, 12 and most contain 1 character beyond the integer
         * or floating point number in question (ie *ymin*, == 5,) (ie *confidence*} == .99})
         * 
         * The two exceptions to the aboce are all instances of xmax "*xmax*}," and the final instance
         * of confidence in a file "*confidence*}]"
         **/
        Scanner entityScanner = new Scanner(entityJson);
        String temp = ""; // Overwritten to store a read String for extra character removal
        
        entityScanner.next();

        // Skips label because the AI is currently color-blind \\
        // (Later this will be used to avoid cargo of opposing alliance)
        entityScanner.next();

        entityScanner.next();
        entityScanner.next();

        // Handles Y \\
        temp = entityScanner.next(); // Reads in ymin
        temp = temp.substring(0,temp.length()-1); // Removes extraneous character
        entityData[1] = (double)Integer.parseInt(temp);// Assigns y

        entityScanner.next();

        // Handles X \\
        temp = entityScanner.next(); // Reads in xmin
        temp = temp.substring(0,temp.length()-1); // Removes extraneous character
        entityData[0] = (double)Integer.parseInt(temp);// Assigns x

        entityScanner.next();

        // Handles Height \\
        temp = entityScanner.next(); // Reads in ymax
        temp = temp.substring(0,temp.length()-1); // Removes extraneous character
        entityData[3] = (double)Integer.parseInt(temp) - entityData[1];// Assigns height

        entityScanner.next();

        // Handles Length \\
        temp = entityScanner.next(); // Reads in xmax
        temp = temp.substring(0,temp.length()-1); // Removes extraneous "}" character
        temp = temp.substring(0,temp.length()-1); // Removes extraneous "," character
        entityData[2] = (double)Integer.parseInt(temp) - entityData[0];// Assigns length

        entityScanner.next();

        // Handles Confidence \\
        temp = entityScanner.next(); // Reads in confidence
        // The last confidence of the last potential ball will be followed by a } and a ]
        temp = temp.substring(0,temp.length()-1); // Sometimes removes a desired character, but confidence percision is high enough that this is negligable
        temp = temp.substring(0,temp.length()-1); // Removes extraneous character
        entityData[4] = Double.parseDouble(temp);// Assigns confidence

        // Closes the scanner utilized
        entityScanner.close();

        return entityData;
    }
}
