package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BallTrackingSystem {
    public NetworkTableEntry ballInfo;
    public double diagonalCameraFOV, diagonalCameraResolution;
    
    public BallTrackingSystem(double diagonalCameraFOV, double diagonalCameraResolution) {
        this.diagonalCameraFOV = diagonalCameraFOV;
        this.diagonalCameraResolution = diagonalCameraResolution;
    
        ballInfo = NetworkTableInstance.getDefault().getTable("ML").getEntry("detections");
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
         * {"label": *label*,
         *  "box": {
         *      "ymin": *ymin*,
         *      "xmin": *xmin*,
         *      "ymax": *ymax*,
         *      "xmax": *xmax*,
         *  },
         *  "confidence": *confidence*}
         * 
         * This means that an entity is 8 lines long, the only non-entity portions
         * of an axon json are the leading and tailing square brackets
         **/
        Scanner jsonScanner = new Scanner(axonJson);
        // Burns the leading bracket
        jsonScanner.nextLine();

        // Due to the nature of this loop, the trailing bracket will be read but won't be used
        while(jsonScanner.hasNextLine()) {
            // Check if you have already parsed a full entity
            if(temp.size() == 8) {
                // Reconstruct entity with original newlines
                String entity = "";

                for(String entityPart : temp) {
                    entity += entityPart +"\n";
                }

                output.add(entity);
                // Reset temp
                temp.clear();
            }

            // Read the next line of the json
            temp.add(jsonScanner.nextLine());
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
         * {"label": *label*,
         *  "box": {
         *      "ymin": *ymin*,
         *      "xmin": *xmin*,
         *      "ymax": *ymax*,
         *      "xmax": *xmax*,
         *  },
         *  "confidence": *confidence*}
         * 
         * thus breaking the string on whitespace yeilds the sequence:
         * "label":
         * *label*,
         * "box":
         * {
         * "ymin":
         * *ymin*,
         * "xmin":
         * *xmin,
         * ...
         * 
         * Thus the terms of interest are at indicies 1, 5, 7, 9, 11, 14 and all contain 1 character beyond the integer
         * or floating point number in question (ie *ymin*, == 5,) (ie *confidence*} == .99})
         **/
        Scanner entityScanner = new Scanner(entityJson);
        String temp = ""; // Overwritten to store a read String for extra character removal
        
        entityScanner.next();

        // Skips label because the AI is currently color-blind \\
        // (Later this will be used to avoid cargo of opposing alliance)
        entityScanner.next();

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
        temp = temp.substring(0,temp.length()-1); // Removes extraneous character
        entityData[2] = (double)Integer.parseInt(temp) - entityData[0];// Assigns length

        entityScanner.next();
        entityScanner.next();

        // Handles Confidence \\
        temp = entityScanner.next(); // Reads in confidence
        temp = temp.substring(0,temp.length()-1); // Removes extraneous character
        entityData[4] = (double)Double.parseDouble(temp) - entityData[0];// Assigns confidence

        // Closes the scanner utilized
        entityScanner.close();

        return entityData;
    }
}
