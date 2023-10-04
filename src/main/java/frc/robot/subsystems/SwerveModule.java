package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.util.ModuleStateOptimizer;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import frc.lib.util.CANSparkMaxUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule{
  private int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;
  private double desiredSpeed;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;
  
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private DutyCycleEncoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    
    /* Angle Encoder Config */
    angleEncoder = new DutyCycleEncoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle); 
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  // use to calculate feed forward values
  /*/
  private double tmpTime = System.currentTimeMillis();;
  private double tmpVel = 0.0;
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double voltage = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed * 12.0;
    driveMotor.setVoltage(voltage);

    double accel = (driveEncoder.getVelocity()-tmpVel)/((System.currentTimeMillis()-tmpTime)/1000.0);

    if (moduleNumber == 0 && voltage > 0 && accel > 0 && driveEncoder.getVelocity() > 0) {
      double err = voltage - (Constants.Swerve.driveKA + Constants.Swerve.driveKV * driveEncoder.getVelocity() + Constants.Swerve.driveKA * accel);
      System.out.format("Voltage: %.3f Velocity: %.4f Acceleration: %.5f Err: %.3f\n", voltage, driveEncoder.getVelocity(), accel, err);
    }
    tmpTime = System.currentTimeMillis();
    tmpVel = driveEncoder.getVelocity();
    
  }*/

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredSpeed = desiredState.speedMetersPerSecond;
    if(isOpenLoop){
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    }
    else {
      driveController.setReference(
        desiredState.speedMetersPerSecond,
        CANSparkMax.ControlType.kVelocity,
        0,
        feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    angleController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder(){
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition());
  }

  public double getDesiredSpeed() {
    return desiredSpeed;
  }

  public void resetToAbsolute(){
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();

    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder(){     
    angleEncoder.reset();
  }

  private void configAngleMotor(){
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor(){        
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.driveKP); //TODO: find why this is angleKP, I, and D or fix
    driveController.setI(Constants.Swerve.driveKI);
    driveController.setD(Constants.Swerve.driveKD);
    driveController.setFF(Constants.Swerve.driveKF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public int getModuleNumber() {
    return moduleNumber;
  }
}