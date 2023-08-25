package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.sensors.IMU;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private static Swerve instance;
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private IMU imu;
    private DoubleLogEntry mod0DriveVel, mod0Angle;
    private DoubleLogEntry mod1DriveVel, mod1Angle;
    private DoubleLogEntry mod2DriveVel, mod2Angle;
    private DoubleLogEntry mod3DriveVel, mod3Angle;

    private Swerve() {
        imu = IMU.getInstance();
        zeroGyro();
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0); //TODO: see if I still need this with rev
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        initializeLogger();
    }

    private void initializeLogger() {
        mod0Angle    = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod0/Angle");
        mod0DriveVel = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod0/DriveVel");
        mod1Angle    = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod1/Angle");
        mod1DriveVel = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod1/DriveVel");
        mod2Angle    = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod2/Angle");
        mod2DriveVel = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod2/DriveVel");
        mod3Angle    = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod3/Angle");
        mod3DriveVel = new DoubleLogEntry(DataLogManager.getLog(), "/Swerve/Mod3/DriveVel");
    }

    private void logPeriodic() {
        SwerveModuleState[] states = getModuleStates();
        mod0Angle.append(states[0].angle.getDegrees());
        mod0DriveVel.append(states[0].speedMetersPerSecond);
        mod1Angle.append(states[1].angle.getDegrees());
        mod1DriveVel.append(states[1].speedMetersPerSecond);
        mod2Angle.append(states[2].angle.getDegrees());
        mod2DriveVel.append(states[2].speedMetersPerSecond);
        mod3Angle.append(states[3].angle.getDegrees());
        mod3DriveVel.append(states[3].speedMetersPerSecond);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        imu.setOffset(imu.getHeading());
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - imu.getHeading()) : Rotation2d.fromDegrees(imu.getHeading());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public static Swerve getInstance(){
        if(instance == null){
            instance = new Swerve();
        }
        return instance;
    }
}