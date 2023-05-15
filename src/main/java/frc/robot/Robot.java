// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.statemachine.Controllers;
import frc.robot.statemachine.RobotStateManager;
import frc.robot.states.AutoState;
import frc.robot.states.DisabledState;
import frc.robot.states.TeleOpState;
import frc.robot.states.TestState;

public class Robot extends TimedRobot {
  private Controllers controllers;
  private RobotStateManager stateMachine;

  @Override
  public void robotInit() {
    controllers = Controllers.getInstance();
    stateMachine = RobotStateManager.getInstance();

    stateMachine.addStates(
      new DisabledState("disabled", null),
      new AutoState("auto", null),
      new TeleOpState("teleOp", null),
      new TestState("test", null)
    );
  }

  @Override
  public void robotPeriodic() {
    stateMachine.periodic();
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
  public void teleopPeriodic() {
    controllers.eventPeriodic();
  }

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
