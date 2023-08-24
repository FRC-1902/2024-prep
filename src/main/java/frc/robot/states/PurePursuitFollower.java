package frc.robot.states;

import frc.robot.statemachine.State;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.sensors.IMU;
import frc.robot.Constants;
import frc.robot.statemachine.RobotStateManager;

public class PurePursuitFollower implements State{
    private String name;
    private String parent;
    private SwerveDrivePoseEstimator poseEstimator;
    private IMU imu;
    private Swerve swerveSubsystem;
    private Waypoints waypoints;

    private static final String FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";
    
    public PurePursuitFollower(String name, String parent){
        this.name = name;
        this.parent = parent;
        imu = IMU.getInstance();
        swerveSubsystem = Swerve.getInstance();
    }
    
    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getParent() {
        return parent;
    }

    @Override
    public void enter() {
        Rotation2d initialAngle = Rotation2d.fromDegrees(imu.getHeading());
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            initialAngle, 
            swerveSubsystem.getModulePositions(), 
            new Pose2d(0.0, 0.0, initialAngle)
        );
        waypoints = new Waypoints(FILEPATH + "TestPath1.csv");
    }

    @Override
    public void leave() {}

    @Override
    public void periodic(RobotStateManager rs) {
        poseEstimator.update(Rotation2d.fromDegrees(imu.getHeading()), swerveSubsystem.getModulePositions());
        pursuit(poseEstimator.getEstimatedPosition());
    }

    private void pursuit(Pose2d estimatedPose) {
        double searchDistance = 0.1; //TODO: tune and migrate to constants
        double targetVelocity = waypoints.findClosestVelocity(estimatedPose);
        Rotation2d targetFacingAngle = waypoints.findClosestPose(estimatedPose).getRotation();
        Pose2d lookahead = waypoints.findLookahead(estimatedPose, searchDistance);
        if(lookahead == null)
            lookahead = waypoints.findClosestPose(estimatedPose);

        // rotation between current and target
        Rotation2d driveAngle = Rotation2d.fromRadians(
            0.0// TODO: write me
        );

        Translation2d driveTarget = new Translation2d(targetVelocity, driveAngle);

        swerveSubsystem.drive(
            driveTarget,
            0.0, //TODO: implement rotation controller for facingAngle
            true,
            false
        );
        // ending handling? (test without first)

        logPursuit(estimatedPose, lookahead, driveTarget, targetFacingAngle);
    }

    private void logPursuit(Pose2d estimatedPose, Pose2d lookahead, Translation2d driveTarget, Rotation2d targetFacingAngle) {
        // TODO: write me
        // make compatible with advantage scope
    }
}

class Waypoints {
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
}
