package frc.robot.states;

import frc.robot.statemachine.State;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.sensors.IMU;
import frc.robot.Constants;
import frc.robot.statemachine.RobotStateManager;
import frc.lib.util.Waypoints;

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