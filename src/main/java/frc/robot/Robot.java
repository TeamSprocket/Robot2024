// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotState;
// import frc.robot.commands.macro.FollowPath;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Timer.delay(0.5);
    m_robotContainer.getSwerveDrive().initGyro();
    m_robotContainer.getSwerveDrive().resetModulesToAbsolute();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Constants.robotState = RobotState.DISABLED;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Constants.robotState = RobotState.AUTON;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    Timer.delay(0.5);
    m_robotContainer.getSwerveDrive().initGyro();
    m_robotContainer.getSwerveDrive().resetModulesToAbsolute();
    m_robotContainer.getSwerveDrive().zeroDriveMotors();;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Constants.robotState = RobotState.TELEOP;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
