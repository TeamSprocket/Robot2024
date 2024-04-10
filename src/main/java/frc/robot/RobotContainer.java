// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.DoNothing;
import frc.robot.auto.OneNoteNoTaxi;
import frc.robot.commands.EjectNote;
import frc.robot.commands.auto.IntakeNoteManualTimed;
// import frc.robot.commands.auto.OneNoteAuton;
import frc.robot.commands.auto.ScoreSpeakerSubwooferShootTimed;
import frc.robot.commands.auto.ScoreSpeakerSubwooferSpinupTimed;
import frc.robot.commands.instant.*;
import frc.robot.commands.macro.LockHeadingToSpeaker;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.controls.Controller;
// import frc.robot.commands.macro.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final static Controller driver = new Controller(0);
  public final static Controller operator = new Controller(1);

  PowerDistribution pdh = new PowerDistribution();

  Vision limelight = new Vision();
  SwerveDrive swerveDrive = new SwerveDrive(limelight);
  Elevator elevator = new Elevator();
  ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getController().getLeftY(), () -> swerveDrive.getTranslation3d());
  Shooter shooter = new Shooter(() -> swerveDrive.getPose().getTranslation(), () -> operator.getController().getRightTriggerAxis(), () -> operator.getController().getLeftTriggerAxis(), () -> swerveDrive.getTranslation3d());
  Intake intake = new Intake();

  

  // Superstructure superstructure = new Superstructure(elevator, shooterPivot, shooter, intake);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }

  public void initAutons() {
    autonChooser.setDefaultOption("Do Nothing", new DoNothing());
    // autonChooser.addOption("One Note No Taxi", new OneNoteNoTaxi(shooter, intake));
    
    // autonChooser.addOption("Figure Eight Test", new PathPlannerAuto("FigEightTestAuton"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 4Note"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("Intake Test"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 8Note"));
    autonChooser.addOption("ANY: Taxi", new PathPlannerAuto("ANY Taxi"));
    autonChooser.addOption("ANY 1Note", new PathPlannerAuto("ANY 1Note"));
    // autonChooser.addOption("B2: One Note", new PathPlannerAuto("B2 1Note"));
    // autonChooser.addOption("PrintHello", new PathPlannerAuto("TestingNamedCommands"));
    // autonChooser.addOption("PPTranslationTuning", new PathPlannerAuto("PPTranslationTuning"));
    // autonChooser.addOption("Just Moving", new PathPlannerAuto("JustTranslation"));
    autonChooser.addOption("B2 2Note", new PathPlannerAuto("B2 2Note"));
    // autonChooser.addOption("B2 3Note", new PathPlannerAuto("B2 3Note")); 

    // autonChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public void initNamedCommands() {
    NamedCommands.registerCommand("IntakeNote", new IntakeNoteManual(intake, shooter, shooterPivot));
    NamedCommands.registerCommand("SpinupSubwoofer", new ScoreSpeakerSubwooferSpinupTimed(shooter, 3.0));
    NamedCommands.registerCommand("ShootSubwoofer", new ScoreSpeakerSubwooferShoot(shooter, intake));
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
    // driver.rightBumper().onTrue(new ZeroGyro(swerveDrive));
    driver.getController().rightBumper().onTrue(new InstantCommand(swerveDrive::zeroHeading)); // hehe it's faster :3
    // driver.leftBumper().onTrue(new LockHeadingToSpeaker());
    
    
    // driver.leftBumper().onTrue(new AlignWithAprilTag(swerveDrive));
    // driver.button(RobotMap.Controller.Y)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.FORWARD));
    // driver.button(RobotMap.Controller.X)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.LEFT));
    // driver.button(RobotMap.Controller.B)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.RIGHT));
    // driver.button(RobotMap.Controller.A)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.BACK));

    // --------------------=operator=--------------------
    operator.getController().leftBumper().whileTrue(new ScoreAmp(elevator, shooterPivot, shooter));
    operator.getController().rightBumper().whileTrue(new ScoreSpeakerSubwooferShoot(shooter, intake));

    operator.getController().y().whileTrue(new WaitAmp(elevator, shooterPivot));
    operator.getController().x().whileTrue(new ScoreSpeakerSubwooferSpinup(shooter));
    // operator.b().whileTrue(new ScoreSpeakerAmpZoneSpinup(intake, shooter, shooterPivot));
    operator.getController().a().whileTrue(new IntakeNoteManual(intake, shooter, shooterPivot));

    operator.getController().button(RobotMap.Controller.VIEW_BUTTON).whileTrue(new EjectNote(intake, shooter, shooterPivot)); // View button
    operator.getController().button(RobotMap.Controller.MENU_BUTTON).whileTrue(new ShootCrossfield(shooterPivot, shooter, intake)); // Menu button

    operator.getController().button(RobotMap.Controller.LEFT_STICK_BUTTON).whileTrue(new ReIndexNote(shooter, shooterPivot));

  }

  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
  }

  public void updateNoteRumbleListener() {
    driver.updateNoteRumbleListener(shooter::beamBroken, shooter::getState);
    operator.updateNoteRumbleListener(shooter::beamBroken, shooter::getState);
  }

  public void stopNoteRumbleListener() {
    driver.stopNoteRumbleListener();
    operator.stopNoteRumbleListener();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  
  public ShooterPivot getShooterPivot() {
    return shooterPivot;
  }

  public void clearPDHStickyFaults() {
    pdh.clearStickyFaults();
  }
  

  

  


  

}
