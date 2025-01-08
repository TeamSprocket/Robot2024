// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.macro.FollowAprilTag;
import frc.robot.commands.persistent.*;
import frc.robot.controls.Controller;
// import frc.robot.commands.macro.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final static Controller driver = new Controller(0);
  public final static Controller operator = new Controller(1);

  PowerDistribution pdh = new PowerDistribution();

  Vision limelight;

  SwerveDrive swerveDrive = new SwerveDrive(limelight);


  // Superstructure superstructure = new Superstructure(elevator, shooterPivot, shooter, intake);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }

  public void initAutons() {
    
    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public void initNamedCommands() {

    // NamedCommands.registerCommand("ScoreSpeaker", new ScoreSpeaker(superstructure, swerveDrive));
    // NamedCommands.registerCommand("ScoreSpeakerSubwooferShoot", new ScoreSpeakerSubwooferShoot(shooter, intake));
    // NamedCommands.registerCommand("PrintHello", new PrintHello());

  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
    // return new OneNoteAuton(shooter, intake);
  }

  public void configureBindings() {
    // --------------------=Driver=--------------------
    swerveDrive.setDefaultCommand(new DriveTeleop(
        swerveDrive,
        () -> -driver.getController().getLeftX(),
        () -> driver.getController().getLeftY(),
        () -> -driver.getController().getRightX()));
    
    
    driver.getController().leftBumper().onTrue(new FollowAprilTag(swerveDrive));

  }

  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
  }

  public void updateNoteRumbleListener() {
  }



  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void clearPDHStickyFaults() {
    pdh.clearStickyFaults();
  }

  public void zeroSuperstructurePositions() {
    
  }

}
