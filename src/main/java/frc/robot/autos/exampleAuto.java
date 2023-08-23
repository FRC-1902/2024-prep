package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

//TODO: remove command auto, look at other team's auto implementations
public class exampleAuto extends SequentialCommandGroup {
    private static final String FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";
    private static final String CSV_DELIMITER = ",";

    public exampleAuto(Swerve swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                poseGenerator("TestPath1.csv"),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        //TODO: this won't do anything as I removed the command scheduler
        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }

    //Convert pathplanner JSON to wpiLib Trajectory
    //TODO: find a better way to do this, points grabbed along splines and fed into a spline trajectory isn't great
    private List<Pose2d> poseGenerator(String fileName){
        List<Pose2d> poses = new ArrayList<>();
        
        try(BufferedReader br = new BufferedReader(new FileReader(FILEPATH + fileName))) {
            // Skip first 2 lines of CSV
            br.readLine();
            br.readLine();

            String line;
            while((line = br.readLine()) != null) {
                String[] cols = line.split(CSV_DELIMITER);
                poses.add(new Pose2d(Double.parseDouble(cols[1]), 
                    Double.parseDouble(cols[2]), 
                    new Rotation2d(Double.parseDouble(cols[7]) * Math.PI / 180.0)));
            }
        } catch(Exception e) {
            e.printStackTrace();
            DataLogManager.log("ERROR! file read error: " + FILEPATH + fileName);
            // make blank pose list so that no error occurs on trajectory parsing
            // can't be too small or TragectoryGenerator errors out
            poses = List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        }
        return poses;
    }
}