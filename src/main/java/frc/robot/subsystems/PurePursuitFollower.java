package frc.robot.subsystems;

import java.util.ArrayDeque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.sensors.IMU;
import frc.robot.Constants;
import frc.lib.util.Waypoints;

public class PurePursuitFollower{
    private IMU imu;
    private Swerve swerveSubsystem;

    private SwerveDrivePoseEstimator poseEstimator;
    private PIDController anglePID;
    private int countAtSetpoint;
    private boolean isFinished;

    private ArrayDeque<Waypoints> waypointsQueue;
    private Waypoints activeWaypoints;
    
    public PurePursuitFollower(){
        imu = IMU.getInstance();
        swerveSubsystem = Swerve.getInstance();
        anglePID = new PIDController(Constants.AutoConstants.ANGLE_KP, Constants.AutoConstants.ANGLE_KI, Constants.AutoConstants.ANGLE_KD);
        anglePID.setTolerance(Constants.AutoConstants.TARGET_ANGLE_DELTA);
        isFinished = true;
        waypointsQueue = new ArrayDeque<>();
    }

    public void queueWaypoint(Waypoints newWaypoint) {
        waypointsQueue.add(newWaypoint);
    }

    /**
     * Call to activate the next waypoint in the queue
     */
    public void start() {
        if (waypointsQueue.isEmpty()) {
            return;
        }

        Rotation2d initialAngle = imu.getHeading();
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            initialAngle, 
            swerveSubsystem.getModulePositions(), 
            new Pose2d(0.0, 0.0, initialAngle)
        );

        activeWaypoints = waypointsQueue.poll();
        countAtSetpoint = 0;
        isFinished = false;

        DataLogManager.log("Starting Next Path");
    }

    /**
     * Stops the drivetrain and sets isFinished to true
     */
    private void leave() {
        swerveSubsystem.drive(
            new Translation2d(0.0, 0.0),
            0.0,
            true,
            false
        );
        
        isFinished = true;

        DataLogManager.log("Finished Queued Path");
    }

    public boolean isFinished() {
        return isFinished;
    }

    /**
     * Call every robot loop until finished
     */
    public void periodic() {
        if (!isFinished) {
            poseEstimator.update(imu.getHeading(), swerveSubsystem.getModulePositions());
            pursuit(poseEstimator.getEstimatedPosition());
        }
    }

    /**
     * Pursue from estimated pose to active waypoints
     * @param estimatedPose
     */
    private void pursuit(Pose2d estimatedPose) {
        // exit handling
        double endDistance = activeWaypoints.getEndpoint().getTranslation().getDistance(estimatedPose.getTranslation());
        if(endDistance < Constants.AutoConstants.TARGET_END_DELTA && anglePID.atSetpoint() || isFinished) { 
            countAtSetpoint += 1;
            if (countAtSetpoint >= Constants.AutoConstants.TARGET_COUNT_AT_SETPIONT || isFinished) {
                leave();
                return;
            }
        } else {
            countAtSetpoint = 0;
        }

        // get targets
        double targetVelocity = activeWaypoints.findClosestVelocity(estimatedPose);
        Rotation2d targetFacingAngle = activeWaypoints.findClosestPose(estimatedPose).getRotation();
        Rotation2d currentFacingAngle = imu.getHeading();
        Pose2d lookahead = activeWaypoints.findLookahead(estimatedPose, Constants.AutoConstants.SEARCH_DISTANCE);
        if(lookahead == null)
            lookahead = activeWaypoints.findClosestPose(estimatedPose);

        // rotation between current and target
        Rotation2d driveAngle = activeWaypoints.facePoint(estimatedPose.getTranslation(), lookahead.getTranslation());

        // XXX: may need to rexamine later, possibly removing initial acceleration from waypoint generation
        targetVelocity = Math.abs(targetVelocity);
        targetVelocity = Math.max(targetVelocity, Constants.AutoConstants.MIN_VELOCITY);

        // try and hit targets with swerve
        Translation2d driveTarget = new Translation2d(targetVelocity, driveAngle);
        swerveSubsystem.drive(
            driveTarget,
            angleControl(currentFacingAngle, targetFacingAngle),
            true,
            false
        );
    }

    /**
     * Cascading PID angle controller with error limiting to avoid instability
     * @param current
     * @param target
     * @return
     */
    private double angleControl(Rotation2d current, Rotation2d target) {
        Rotation2d error = current.minus(target);
        double adjustedError = Math.abs(error.getDegrees()) < Constants.AutoConstants.ANGLE_ERROR_LIMIT ?
            error.getDegrees() : Constants.AutoConstants.ANGLE_ERROR_LIMIT * Math.signum(error.getDegrees());
        return anglePID.calculate(adjustedError, 0.0);
    }
}