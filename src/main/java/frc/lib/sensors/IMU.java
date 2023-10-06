package frc.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class IMU {

  private static IMU instance = new IMU();

  private final BNO055 bno055Euler = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
    BNO055.vector_type_t.VECTOR_EULER);
  // only velocity or acceleration can be used at once
  // private final BNO055 bno055Accel = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
  //   BNO055.vector_type_t.VECTOR_LINEARACCEL);

  private DoubleLogEntry headingLogger, rollLogger, pitchLogger, turnLogger, offsetLogger; 

  private void initializeLogger() {
    headingLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/heading");
    rollLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/roll");
    pitchLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/pitch");
    turnLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/turn");
    offsetLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/offset");

    offsetLogger.append(bno055Euler.headingOffset);
  }

  public IMU() {
    initializeLogger();
  }

  public void logPeriodic() {
    headingLogger.append(getHeading().getDegrees());
    rollLogger.append(getRoll());
    pitchLogger.append(getPitch());
    turnLogger.append(getTurns());
  }

  /**
   * @return returns the imu's x scalar (heading/yaw) as a Rotation2d object
   */
  public Rotation2d getHeading() {
    double[] xyz = bno055Euler.getVector();
    return (Constants.Swerve.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - xyz[0]) : Rotation2d.fromDegrees(xyz[0]);
  }

  /**
   * @return returns the imu's y scalar (roll) representing an angle from -90 to
   *         90 degrees
   */
  public double getRoll() {
    double[] xyz = bno055Euler.getVector();
    return xyz[1];
  }

  /**
   * @return returns the imu's z scalar (pitch) representing an angle from -180 to
   *         180 degrees
   */
  public double getPitch() {
    double[] xyz = bno055Euler.getVector();
    return xyz[2];
  }

  /**
   * @return the signed sum of the amount of full rotations the BNO has taken
   */
  public long getTurns() {
    return bno055Euler.getTurns();
  }

  /**
   * @param offset sets imu x heading offset
   */
  public void setOffset(double offset) {
    bno055Euler.headingOffset = offset;
    offsetLogger.append(offset);
  }

  /**
   * resets imu x heading to default offset
   */
  public void resetHeading() {
    bno055Euler.resetHeading();
  }

  /**
   * @return true if the sensor is found on the I2C bus
   */
  public boolean isSensorPresent() {
    return bno055Euler.isSensorPresent();
  }

  /**
   * @return true when the sensor is initialized.
   */
  public boolean isInitialized() {
    return bno055Euler.isInitialized();
  }

  /**
   * @return true if calibration is complete for all sensors required for the
   *         mode the sensor is currently operating in.
   */
  public boolean isCalibrated() {
    return bno055Euler.isCalibrated();
  }

  /**
   * @return imu instance
   */
  public static IMU getInstance() {
    if (instance == null) {
      instance = new IMU();
    }
    return instance;
  }
}