package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Swerve;
import frc.lib.util.Waypoints;

// TODO: testme after changes

public class PurePursuitCommand extends Command{
    private IMU imu;
    private Swerve swerveSubsystem;

    private PIDController anglePID;
    private int countAtSetpoint;

    private Waypoints waypoints;
    private Pose2d startingPose;

    private static final String LOG_KEY = "Pathing";
    
    /**
     * Pathing command factory
     * @param waypoints waypoints path to follow
     */
    public PurePursuitCommand(Waypoints waypoints){
        this.waypoints = waypoints;
        imu = IMU.getInstance();
        swerveSubsystem = Swerve.getInstance();
        anglePID = new PIDController(Constants.AutoConstants.ANGLE_KP, Constants.AutoConstants.ANGLE_KI, Constants.AutoConstants.ANGLE_KD);
        anglePID.setTolerance(Constants.AutoConstants.TARGET_ANGLE_DELTA);
    }

    /**
     * Call to activate the next waypoint
     */
    @Override
    public void initialize() {
        startingPose = swerveSubsystem.getPose();

        countAtSetpoint = 0;

        Logger.recordOutput(LOG_KEY, "Starting Next Path");
    }

    /**
     * Stops the drivetrain
     */
    @Override 
    public void end(boolean wasInterrupted) {
        swerveSubsystem.drive(
            new Translation2d(0.0, 0.0),
            0.0,
            true,
            false
        );

        if (!wasInterrupted) {
            Logger.recordOutput(LOG_KEY, "Finished Queued Path");
        } else {
            Logger.recordOutput(LOG_KEY, "Path Was Interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        double endDistance = waypoints.getEndpoint().getTranslation().getDistance(
            getLocalPose().getTranslation()
        );
        if(endDistance < Constants.AutoConstants.TARGET_END_DELTA && anglePID.atSetpoint()) { 
            countAtSetpoint += 1;
            if (countAtSetpoint >= Constants.AutoConstants.TARGET_COUNT_AT_SETPIONT) {
                return true;
            }
        } else {
            countAtSetpoint = 0;
        }
        return false;
    }

    /**
     * Call every robot loop until finished
     */
    @Override
    public void execute() {
        // get targets
        double targetVelocity = waypoints.findClosestVelocity(getLocalPose());
        Rotation2d targetFacingAngle = waypoints.findClosestPose(getLocalPose()).getRotation();
        Rotation2d currentFacingAngle = imu.getHeading();
        Pose2d lookahead = waypoints.findLookahead(getLocalPose(), Constants.AutoConstants.SEARCH_DISTANCE);
        if(lookahead == null)
            lookahead = waypoints.findClosestPose(getLocalPose());

        // rotation between current and target
        Rotation2d driveAngle = waypoints.facePoint(getLocalPose().getTranslation(), lookahead.getTranslation());

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

    /**
     * 
     */
    private Pose2d getLocalPose() {
        return swerveSubsystem.getPose().relativeTo(startingPose);
    }
}