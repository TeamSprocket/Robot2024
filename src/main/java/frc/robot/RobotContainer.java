// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.instant.*;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
// import frc.robot.commands.macro.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  public final static CommandXboxController secondary = new CommandXboxController(1);

  Vision vision = new Vision();
  SwerveDrive swerveDrive = new SwerveDrive(vision);



  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();


  public RobotContainer() {
    configureBindings();
    initAutons();
    initNamedCommands();
  }

  

  public void initAutons() {
    autonChooser.addOption("Figure Eight Test",  new PathPlannerAuto("FigEightTestAuton"));
    autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 4Note"));
    autonChooser.addOption("B1: Four Note", new PathPlannerAuto("Intake Test"));
    autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 8Note"));
    
    autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public void initNamedCommands() {
    // NamedCommands.registerCommand("IntakeNote", new IntakeNote(superstructure, swerveDrive));
    // NamedCommands.registerCommand("ScoreSpeaker", new ScoreSpeaker(superstructure, swerveDrive));
  }


  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  
  public void configureBindings() {
    // --------------------=Driver=--------------------
		swerveDrive.setDefaultCommand(new DriveTeleop(
			swerveDrive, 
			() -> -driver.getLeftX(), 
			() -> driver.getLeftY(), 
			() -> -driver.getRightX()));
    driver.x().onTrue(new ZeroGyro(swerveDrive));
    driver.button(RobotMap.Controller.Y).onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.FORWARD));
    driver.button(RobotMap.Controller.X).onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.LEFT));
    driver.button(RobotMap.Controller.B).onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.RIGHT));
    driver.button(RobotMap.Controller.A).onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.BACK));

    // --------------------=Secondary=--------------------
    
  
  }
  
  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
  }   

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

}
