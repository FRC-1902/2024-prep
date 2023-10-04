// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.sensors.IMU;
import frc.lib.statemachine.RobotStateManager;
import frc.robot.states.AutoState;
import frc.robot.states.DisabledState;
import frc.robot.states.PurePursuitFollower;
import frc.robot.states.TeleOpState;
import frc.robot.states.TestState;

public class Robot extends TimedRobot {
  private RobotStateManager stateMachine;
  private IMU imu;

  @Override
  public void robotInit() {
    stateMachine = RobotStateManager.getInstance();
    imu = IMU.getInstance();

    stateMachine.addStates(
      new DisabledState("disabled", null),
      new AutoState("auto", null),
      new TeleOpState("teleOp", null),
      new TestState("test", null),
      new PurePursuitFollower("purePursuit", "auto")
    );
  }

  @Override
  public void robotPeriodic() {
    stateMachine.periodic();
    imu.logPeriodic();
  }

  @Override
  public void disabledInit() {
    stateMachine.setState("disabled");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    stateMachine.setState("auto");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    stateMachine.setState("teleOp");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    stateMachine.setState("test");
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
