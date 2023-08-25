package frc.lib.sensors;

import javax.accessibility.AccessibleExtendedComponent;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class IMU {

  private static IMU instance = new IMU();

  private final BNO055 bno055Euler = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
    BNO055.vector_type_t.VECTOR_EULER);
  private final BNO055 bno055Accel = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
    BNO055.vector_type_t.VECTOR_LINEARACCEL);

  private DoubleLogEntry headingLogger, rollLogger, pitchLogger, turnLogger, offsetLogger, 
    accelXLogger, accelYLogger, accelZLogger;

  private void initializeLogger() {
    headingLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/heading");
    rollLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/roll");
    pitchLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/pitch");
    turnLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/turn");
    offsetLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/offset");
    accelXLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/accelX");
    accelYLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/accelY");
    accelZLogger = new DoubleLogEntry(DataLogManager.getLog(), "/IMU/accelZ");

    offsetLogger.append(bno055Euler.headingOffset);
  }

  public IMU() {
    initializeLogger();
  }

  public void logPeriodic() {
    headingLogger.append(getHeading());
    rollLogger.append(getRoll());
    pitchLogger.append(getPitch());
    turnLogger.append(getTurns());
    
    double[] linearAccelVec = getLinearAccel();
    accelXLogger.append(linearAccelVec[0]);
    accelYLogger.append(linearAccelVec[1]);
    accelZLogger.append(linearAccelVec[2]);
  }

  public double[] getLinearAccel(){
    return bno055Accel.getVector();
  }

  /**
   * @return returns the imu's x scalar (heading/yaw) representing an angle from 0
   *         to 360 degrees
   */
  public double getHeading() {
    double[] xyz = bno055Euler.getVector();
    return xyz[0];
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
   * @reutrn imu instance
   */
  public static IMU getInstance() {
    if (instance == null) {
      instance = new IMU();
    }
    return instance;
  }
}