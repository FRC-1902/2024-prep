// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.sensors.IMU;
import frc.lib.util.OperationMode;
import frc.robot.modes.AutoMode;
import frc.robot.modes.DisabledMode;
import frc.robot.modes.TeleOpMode;
import frc.robot.modes.TestMode;

public class Robot extends TimedRobot {
  private IMU imu;

  private OperationMode disabledMode, autoMode, teleOpMode, testMode;

  @Override
  public void robotInit() {
    imu = IMU.getInstance();

    disabledMode = new DisabledMode();
    autoMode = new AutoMode();
    teleOpMode = new TeleOpMode();
    testMode = new TestMode();
  }

  @Override
  public void robotPeriodic() {
    imu.logPeriodic();
  }

  @Override
  public void disabledInit() {
    disabledMode.enter();
  }

  @Override
  public void disabledPeriodic() {
    disabledMode.periodic();
  }

  @Override
  public void disabledExit() {
    disabledMode.exit();
  }

  @Override
  public void autonomousInit() {
    autoMode.enter();
  }

  @Override
  public void autonomousPeriodic() {
    autoMode.periodic();
  }

  @Override
  public void autonomousExit() {
    autoMode.exit();
  }

  @Override
  public void teleopInit() {
    teleOpMode.enter();
  }

  @Override
  public void teleopPeriodic() {
    teleOpMode.periodic();
  }

  @Override
  public void teleopExit() {
    teleOpMode.exit();
  }

  @Override
  public void testInit() {
    testMode.enter();
  }

  @Override
  public void testPeriodic() {
    testMode.periodic();
  }

  @Override
  public void testExit() {
    testMode.exit();
  }
}
