package frc.robot.states;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.sensors.IMU;
import frc.lib.statemachine.RobotStateManager;
import frc.lib.statemachine.State;
import frc.robot.Constants;
import frc.lib.util.Waypoints;

public class PurePursuitFollower implements State{
    private String name;
    private String parent;
    private SwerveDrivePoseEstimator poseEstimator;
    private IMU imu;
    private Swerve swerveSubsystem;
    private Waypoints waypoints;
    private PIDController anglePID;

    private static final String FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";
    
    public PurePursuitFollower(String name, String parent){
        this.name = name;
        this.parent = parent;
        imu = IMU.getInstance();
        swerveSubsystem = Swerve.getInstance();
        anglePID = new PIDController(Constants.AutoConstants.ANGLE_KP, Constants.AutoConstants.ANGLE_KI, Constants.AutoConstants.ANGLE_KD);
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
        pursuit(poseEstimator.getEstimatedPosition(), rs);
    }

    private void pursuit(Pose2d estimatedPose, RobotStateManager rs) {
        // exit handling
        double endDistance = waypoints.getEndpoint().getTranslation().getDistance(estimatedPose.getTranslation());
        if(endDistance < Constants.AutoConstants.TARGET_END_DELTA) {
            swerveSubsystem.drive(
                new Translation2d(0.0, 0.0),
                0.0,
                true,
                false
            );

            rs.setState(parent);
            return;
        }

        double targetVelocity = waypoints.findClosestVelocity(estimatedPose);
        Rotation2d targetFacingAngle = waypoints.findClosestPose(estimatedPose).getRotation();
        Rotation2d currentFacingAngle = Rotation2d.fromDegrees(imu.getHeading());
        Pose2d lookahead = waypoints.findLookahead(estimatedPose, Constants.AutoConstants.SEARCH_DISTANCE);
        if(lookahead == null)
            lookahead = waypoints.findClosestPose(estimatedPose);

        // rotation between current and target
        Rotation2d driveAngle = waypoints.facePoint(estimatedPose.getTranslation(), lookahead.getTranslation());

        // XXX: may need to rexamine later, possibly removing initial acceleration
        targetVelocity = Math.abs(targetVelocity);
        targetVelocity = Math.max(targetVelocity, Constants.AutoConstants.MIN_VELOCITY);

        Translation2d driveTarget = new Translation2d(targetVelocity, driveAngle);
        
        swerveSubsystem.drive(
            driveTarget,
            0.0, // angleControl(currentFacingAngle, targetFacingAngle), // TODO: tune angle controller before use
            true,
            false
        );

        logPursuit(estimatedPose, lookahead, driveTarget, targetFacingAngle);
    }

    private double angleControl(Rotation2d current, Rotation2d target) {
        Rotation2d error = current.minus(target);
        double adjustedError = Math.abs(error.getDegrees()) < Math.abs(Constants.AutoConstants.ANGLE_ERROR_LIMIT) ?
            error.getDegrees() : Constants.AutoConstants.ANGLE_ERROR_LIMIT;
        return anglePID.calculate(adjustedError, 0.0);
    }

    private void logPursuit(Pose2d estimatedPose, Pose2d lookahead, Translation2d driveTarget, Rotation2d targetFacingAngle) {
        // TODO: write me
        // make compatible with advantage scope
    }
}