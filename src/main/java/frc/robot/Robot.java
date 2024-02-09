// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveStates;;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Timer.delay(0.5);
    m_robotContainer.getSwerveDrive().initGyro();
    m_robotContainer.getSwerveDrive().zeroDriveMotors();
    m_robotContainer.getSwerveDrive().resetModulesToAbsolute();
    m_robotContainer.getSwerveDrive().setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    SwerveDrive.setState(SwerveDriveStates.DISABLED);
    m_robotContainer.getSwerveDrive().setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SwerveDrive.setState(SwerveDriveStates.AUTON);
    m_robotContainer.getSwerveDrive().zeroDriveMotors();
    m_robotContainer.getSwerveDrive().initGyro();
    m_robotContainer.getSwerveDrive().setNeutralMode(NeutralMode.Brake);

    Timer.delay(0.1);
    m_robotContainer.getSwerveDrive().resetModulesToAbsolute();
    Timer.delay(0.1);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
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
    SwerveDrive.setState(SwerveDriveStates.TELEOP);
    m_robotContainer.getSwerveDrive().setNeutralMode(NeutralMode.Brake);

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
