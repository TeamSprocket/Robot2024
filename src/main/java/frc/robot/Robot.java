// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotState;
import frc.util.ShuffleboardIO;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    Logger.start();
    m_robotContainer = new RobotContainer();

    Timer.delay(0.5);
    // m_robotContainer.getSwerveDrive().zeroHeading(); //
    // m_robotContainer.getSwerveDrive().zeroDriveMotors(); //
    // m_robotContainer.getSwerveDrive().resetModulesToAbsolute(); //
    // m_robotContainer.getSwerveDrive().setNeutralModeDrive(NeutralModeValue.Brake); //
    // m_robotContainer.getSwerveDrive().setNeutralModeTurn(NeutralModeValue.Coast); //

    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Constants.robotState = RobotState.DISABLED;
    // m_robotContainer.getSwerveDrive().setNeutralMode(NeutralModeValue.Coast); //
    m_robotContainer.getShooterPivot().setNeutralMode(NeutralModeValue.Coast);

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Constants.robotState = RobotState.AUTON;
    // m_robotContainer.zeroSuperstructurePositions();
    // m_robotContainer.getSwerveDrive().zeroDriveMotors(); //
    // m_robotContainer.getSwerveDrive().zeroGyro();
    // m_robotContainer.getSwerveDrive().setNeutralMode(NeutralModeValue.Brake); //
    // m_robotContainer.getSwerveDrive().resetModulesToAbsolute(); //
    Timer.delay(0.05);

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
    Constants.robotState = RobotState.TELEOP;
    // m_robotContainer.getSwerveDrive().setNeutralModeDrive(NeutralModeValue.Coast); //
    // m_robotContainer.getSwerveDrive().setNeutralModeTurn(NeutralModeValue.Brake); //

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.updateNoteRumbleListener();
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
