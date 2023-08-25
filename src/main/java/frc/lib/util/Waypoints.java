package frc.lib.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Waypoints {
    private List<Pose2d> poseList;
    private List<Double> velocityList;

    public Waypoints(String fileName) {
        parseCSV(fileName);
    }

    private void parseCSV(String fileName) {
        // TODO: write me
        // get values from csv
        // map with starting origin of 0,0
        // write poses and velocities to list
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
        return currentPosition.nearest(null);
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
    public Rotation2d facePoint(Pose2d currentPosition, Pose2d targetPosition) {
        return new Rotation2d(
            targetPosition.getX() - currentPosition.getX(),
            targetPosition.getY() - currentPosition.getY()
        );
    }
}
