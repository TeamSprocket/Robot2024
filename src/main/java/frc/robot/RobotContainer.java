// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.DoNothing;
import frc.robot.auto.OneNoteNoTaxi;
import frc.robot.commands.EjectNote;
import frc.robot.commands.auto.IntakeNoteManualTimed;
import frc.robot.commands.auto.ScoreSpeakerSubwooferShootTimed;
import frc.robot.commands.auto.ScoreSpeakerSubwooferSpinupTimed;
import frc.robot.commands.instant.*;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
// import frc.robot.commands.macro.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  public final static CommandXboxController secondary = new CommandXboxController(1);

  PowerDistribution pdh = new PowerDistribution();

  Vision vision = new Vision();
  SwerveDrive swerveDrive = new SwerveDrive(vision);
  // Elevator elevator = new Elevator(() -> secondary.getLeftTriggerAxis(), () -> secondary.getRightTriggerAxis());
  ShooterPivot shooterPivot = new ShooterPivot(() -> secondary.getLeftY(), () -> vision.getDistanceFromTarget());
  Shooter shooter = new Shooter(() -> swerveDrive.getPose().getTranslation(),  () -> vision.getDistanceFromTarget(), () -> secondary.getRightTriggerAxis(), () -> secondary.getLeftTriggerAxis());
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
    
    autonChooser.addOption("Figure Eight Test", new PathPlannerAuto("FigEightTestAuton"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 4Note"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("Intake Test"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 8Note"));
    autonChooser.addOption("ANY: Taxi", new PathPlannerAuto("ANY Taxi"));
    autonChooser.addOption("B2: One Note", new PathPlannerAuto("B2 1Note"));
    autonChooser.addOption("PrintHello", new PathPlannerAuto("TestingNamedCommands"));
    autonChooser.addOption("PPTranslationTuning", new PathPlannerAuto("PPTranslationTuning"));
    autonChooser.addOption("Just Moving", new PathPlannerAuto("JustTranslation"));
    autonChooser.addOption("B2 2Note", new PathPlannerAuto("B2 2Note"));
    autonChooser.addOption("B2 3Note", new PathPlannerAuto("B2 3Note")); 

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
  }

  public void configureBindings() {
    // --------------------=Driver=--------------------
    swerveDrive.setDefaultCommand(new DriveTeleop(
        swerveDrive,
        () -> -driver.getLeftX(),
        () -> driver.getLeftY(),
        () -> -driver.getRightX() * -1));
    // driver.rightBumper().onTrue(new ZeroGyro(swerveDrive));
    driver.rightBumper().onTrue(new InstantCommand(swerveDrive::zeroGyro)); // hehe it's faster :3
    driver.leftBumper().onTrue(new InstantCommand(swerveDrive::resetModulesToAbsolute)); // use for debugging
    
    // driver.leftBumper().onTrue(new AlignWithAprilTag(swerveDrive));
    // driver.button(RobotMap.Controller.Y)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.FORWARD));
    // driver.button(RobotMap.Controller.X)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.LEFT));
    // driver.button(RobotMap.Controller.B)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.RIGHT));
    // driver.button(RobotMap.Controller.A)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.BACK));

    // --------------------=Secondary=--------------------
    secondary.leftBumper().whileTrue(new IntakeNoteManual(intake, shooter, shooterPivot));
    secondary.rightBumper().whileTrue(new ScoreSpeakerSubwooferSpinup(shooter));
    secondary.x().whileTrue(new ScoreSpeakerSubwooferShoot(shooter, intake));
    secondary.b().whileTrue(new EjectNote(intake, shooter, shooterPivot));
  }

  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
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
