package frc.robot.states;

import frc.robot.subsystems.Swerve;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Queue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.sensors.IMU;
import frc.lib.statemachine.RobotStateManager;
import frc.lib.statemachine.State;
import frc.robot.Constants;
import frc.lib.util.Waypoints;

public class PurePursuitFollower implements State{
    private String name;
    private String parent;
    private RobotStateManager rs;

    private SwerveDrivePoseEstimator poseEstimator;
    private IMU imu;
    private Swerve swerveSubsystem;
    private Waypoints currentWaypoints;
    private Queue<Waypoints> queuedWaypoints;
    private PIDController anglePID;
    private int countAtSetpoint;

    private static final String FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";
    
    public PurePursuitFollower(String name, String parent, RobotStateManager rs){
        this.name = name;
        this.parent = parent;
        this.rs = rs;

        imu = IMU.getInstance();
        swerveSubsystem = Swerve.getInstance();
        anglePID = new PIDController(Constants.AutoConstants.ANGLE_KP, Constants.AutoConstants.ANGLE_KI, Constants.AutoConstants.ANGLE_KD);
        anglePID.setTolerance(Constants.AutoConstants.TARGET_ANGLE_DELTA);

        queuedWaypoints = new ArrayDeque<>();
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
        if (queuedWaypoints.isEmpty()) {
            DataLogManager.log("No Queued Waypoints for Pure Pursuit");
            rs.setState("parent");
            return;
        } else {
            currentWaypoints = queuedWaypoints.poll();
        }

        Rotation2d initialAngle = imu.getHeading();
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            initialAngle, 
            swerveSubsystem.getModulePositions(), 
            new Pose2d(0.0, 0.0, initialAngle)
        );

        countAtSetpoint = 0;
    }

    @Override
    public void leave() {
        swerveSubsystem.drive(
            new Translation2d(0.0, 0.0),
            0.0,
            true,
            false
        );

        currentWaypoints = null;
    }

    @Override
    public void periodic() {
        poseEstimator.update(imu.getHeading(), swerveSubsystem.getModulePositions());
        pursuit(poseEstimator.getEstimatedPosition());
    }

    /**
     * Add the next waypoint to use, waypoints added first will be consumed first
     * @param newWaypoint
     */
    public void queueWaypoint(Waypoints newWaypoint) {
        queuedWaypoints.add(newWaypoint);
    }

    /**
     * Add the next waypoints to use
     * @param newWaypoints Collection object to add
     */
    public void queueWaypoints(Collection<Waypoints> newWaypoints) {
        queuedWaypoints.addAll(newWaypoints);
    }

    /**
     * Remove all queued waypoints
     */
    public void flushWaypoints() {
        queuedWaypoints.clear();
    }

    private void pursuit(Pose2d estimatedPose) {
        // exit handling
        double endDistance = currentWaypoints.getEndpoint().getTranslation().getDistance(estimatedPose.getTranslation());
        if(endDistance < Constants.AutoConstants.TARGET_END_DELTA && anglePID.atSetpoint()) { 
            countAtSetpoint += 1;
            if (countAtSetpoint >= Constants.AutoConstants.TARGET_COUNT_AT_SETPIONT) {
                rs.setState(parent);
                return;
            }
        } else {
            countAtSetpoint = 0;
        }

        double targetVelocity = currentWaypoints.findClosestVelocity(estimatedPose);
        Rotation2d targetFacingAngle = currentWaypoints.findClosestPose(estimatedPose).getRotation();
        Rotation2d currentFacingAngle = imu.getHeading();
        Pose2d lookahead = currentWaypoints.findLookahead(estimatedPose, Constants.AutoConstants.SEARCH_DISTANCE);
        if(lookahead == null)
            lookahead = currentWaypoints.findClosestPose(estimatedPose);

        // rotation between current and target
        Rotation2d driveAngle = currentWaypoints.facePoint(estimatedPose.getTranslation(), lookahead.getTranslation());

        // XXX: may need to rexamine later, possibly removing initial acceleration
        targetVelocity = Math.abs(targetVelocity);
        targetVelocity = Math.max(targetVelocity, Constants.AutoConstants.MIN_VELOCITY);

        Translation2d driveTarget = new Translation2d(targetVelocity, driveAngle);
        
        swerveSubsystem.drive(
            driveTarget,
            angleControl(currentFacingAngle, targetFacingAngle),
            true,
            false
        );
    }

    private double angleControl(Rotation2d current, Rotation2d target) {
        Rotation2d error = current.minus(target);
        double adjustedError = Math.abs(error.getDegrees()) < Constants.AutoConstants.ANGLE_ERROR_LIMIT ?
            error.getDegrees() : Constants.AutoConstants.ANGLE_ERROR_LIMIT * Math.signum(error.getDegrees());
        return anglePID.calculate(adjustedError, 0.0);
    }
}