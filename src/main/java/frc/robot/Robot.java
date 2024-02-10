// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotState;
// import frc.robot.commands.macro.FollowPath;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private WPI_TalonFX frontleft = new WPI_TalonFX(23);
  private WPI_TalonFX frontright = new WPI_TalonFX(20);
  private WPI_TalonFX backleft = new WPI_TalonFX(24);
  private WPI_TalonFX backright = new WPI_TalonFX(21);

  private XboxController joystick1 = new XboxController(0);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Timer.delay(0.5);

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

    Timer.delay(0.5);

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
    m_robotContainer.configureBindings();
    frontleft.setNeutralMode(NeutralMode.Brake);
    frontright.setNeutralMode(NeutralMode.Brake);
    backleft.setNeutralMode(NeutralMode.Brake);
    backright.setNeutralMode(NeutralMode.Brake);
    //Try follow next

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double speed = -joystick1.getRawAxis(5) * 0.6;
    double turn = joystick1.getRawAxis(0) * 0.3;

    double left = speed + turn;
    double right = speed - turn;

    frontleft.set(left);
    frontright.set(-right);
    backleft.set(left);
    backright.set(-right);
  }

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
