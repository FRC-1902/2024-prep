package frc.lib.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Waypoints {
    private List<Pose2d> poseList;
    private List<Double> velocityList;

    /**
     * Generate waypoints from .csv file
     * @param fileName
     */
    public Waypoints(String fileName) {
        poseList = new ArrayList<>();
        velocityList = new ArrayList<>();

        parseCSVFile(fileName);
    }

    private void parseCSVFile(String fileName) {
        try (BufferedReader br = Files.newBufferedReader(Paths.get(fileName))) {
            final String DELIMITER = ",";
            
            // skip first 2 descriptor lines
            br.readLine();
            br.readLine();
            
            String line = br.readLine();
            String[] columns = line.split(DELIMITER);

            // grab first x and y to later map pose as origin
            double firstX = Double.parseDouble(columns[1]);
            double firstY = Double.parseDouble(columns[2]);
            poseList.add(new Pose2d(
                0.0, 
                0.0, 
                Rotation2d.fromDegrees(Double.parseDouble(columns[7]))
            ));
            velocityList.add(Double.parseDouble(columns[4]));

            while ((line = br.readLine()) != null) {
                // convert line into columns
                columns = line.split(DELIMITER);

                // map with starting origin of 0,0 into poseList
                poseList.add(new Pose2d(
                    Double.parseDouble(columns[1]) - firstX, 
                    Double.parseDouble(columns[2]) - firstY,
                    Rotation2d.fromDegrees(Double.parseDouble(columns[7]))
                ));
                velocityList.add(Double.parseDouble(columns[4]));
            }
        
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * @param currentPosition estimated current robot position
     * @param searchDistance max meters from waypoint to path to
     * @return last Pose2d in range of search distance, otherwise null
     */
    public Pose2d findLookahead(Pose2d currentPosition, double searchDistance) {
        // loop backward through pose list to get last element that is within range
        for(int i = poseList.size(); i-- > 0; ) {
            if(distanceBetweenPoses(currentPosition, poseList.get(i)) <= searchDistance)
                return poseList.get(i);
        }
        return null;
    }

    /** Pythagorean theorem for distance between two points */
    private double distanceBetweenPoses(Pose2d a, Pose2d b) {
        return Math.sqrt(
            Math.pow(a.getX() - b.getX(), 2) +
            Math.pow(a.getY() - b.getY(), 2)
        );
    }

    /** Find closest Pose2d of waypoint in relation to the current estimated position */
    public Pose2d findClosestPose(Pose2d currentPosition) {
        return currentPosition.nearest(poseList);
    }

    /** Find closest velocity of waypoint in relation to the current estimated position */
    public double findClosestVelocity(Pose2d currentPosition) {
        Pose2d nearestPose = currentPosition.nearest(poseList);
        return velocityList.get(poseList.indexOf(nearestPose));
    }

    /**
     * @param currentPosition
     * @param targetPosition
     * @return Rotation2d to face the target position
     */
    public Rotation2d facePoint(Translation2d currentPosition, Translation2d targetPosition) {
        return new Rotation2d(
            targetPosition.getX() - currentPosition.getX(),
            targetPosition.getY() - currentPosition.getY()
        );
    }

    /**
     * Get the desired final pose from waypoints
     * @return last Pose2d
     */
    public Pose2d getEndpoint() {
        return poseList.get(poseList.size()-1);
    }
}
