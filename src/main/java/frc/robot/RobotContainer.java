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
// import frc.robot.commands.instant.*;
import frc.robot.commands.persistent.*;
// import frc.robot.commands.macro.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  public final static CommandXboxController secondary = new CommandXboxController(1);

  PowerDistribution pdh = new PowerDistribution();

  // Vision vision = new Vision();
  SwerveDrive swerveDrive = new SwerveDrive();
  //Shintake shintake = new Shintake();

  // Elevator elevator = new Elevator(() -> secondary.getLeftTriggerAxis(), () ->
  // secondary.getRightTriggerAxis());
  // ShooterPivot shooterPivot = new ShooterPivot(() -> secondary.getLeftY(), () -> vision.getDistanceFromTarget());
  // swerveDrive.getPose().getTranslation());
  // Shooter shooter = new Shooter(() -> swerveDrive.getPose().getTranslation(),  () -> vision.getDistanceFromTarget());
  // Intake intake = new Intake();

  // Superstructure superstructure = new Superstructure(elevator, shooterPivot, shooter, intake);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }

  public void initAutons() {
    autonChooser.setDefaultOption("Do Nothing", null);
    // autonChooser.addOption("One Note No Taxi", new OneNoteNoTaxi(shooter, intake));
    
    // autonChooser.addOption("Figure Eight Test", new PathPlannerAuto("FigEightTestAuton"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 4Note"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("Intake Test"));
    // autonChooser.addOption("B1: Four Note", new PathPlannerAuto("B1 8Note"));
    autonChooser.addOption("ANY: Taxi", new PathPlannerAuto("ANY Taxi"));    
    // TODO: add none option

    // autonChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public void initNamedCommands() {
    // NamedCommands.registerCommand("IntakeNote", new IntakeNoteManual(intake, shooter, shooterPivot));
    // NamedCommands.registerCommand("SpinupSubwoofer", new ScoreSpeakerSubwooferSpinupTimed(shooter, 3.0));
    // NamedCommands.registerCommand("ShootSubwoofer", new ScoreSpeakerSubwooferShoot(shooter, intake));
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
        () -> -driver.getLeftX(),
        () -> driver.getLeftY(),
        () -> -driver.getRightX() * -1));
    // driver.rightBumper().onTrue(new ZeroGyro(swerveDrive));
    driver.rightBumper().onTrue(new InstantCommand(swerveDrive::zeroGyro)); // hehe it's faster :3
    // driver.leftBumper().onTrue(new InstantCommand(swerveDrive::resetModulesToAbsolute)); // use for debugging
    // driver.a().onTrue(new InstantCommand(vision::setVisionDriverMode));
    
    // driver.leftBumper().onTrue(new AlignWithAprilTag(swerveDrive));
    // driver.button(RobotMap.Controller.Y)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.FORWARD));
    // driver.button(RobotMap.Controller.X)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.LEFT));
    // driver.button(RobotMap.Controller.B)
    //     .onTrue(new SwitchTargetHeadingsDirection(swerveDrive, SwerveDrive.Directions.RIGHT));
    // driver.button(RobotMap.Controller.A)
    //     .onTrue(new SwitchTargetHeadingDirection(swerveDrive, SwerveDrive.Directions.BACK));

    // secondary.leftBumper().whileTrue(new IntakeNoteManual(intake, shooter));
    // secondary.rightBumper().whileTrue(new ScoreSpeakerSubwooferSpinup(shooter));
    // secondary.x().whileTrue(new ScoreSpeakerSubwooferShoot(shooter, intake));
    // secondary.b().whileTrue(new EjectNote(intake, shooter));
    // ACTUAL STUFF:
    // driver.leftBumper().whileTrue(new EjectNote(shintake));
    // driver.rightBumper().whileTrue(new ShintakeIntakeNote(shintake));
    // secondary.rightBumper().whileTrue(new ShintakeScoreAmp(shintake));


    // --------------------=Secondary=--------------------

  }

  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  
  // public ShooterPivot getShooterPivot() {
  //   return shooterPivot;
  // }

  // public Shintake getShintake() {
  //   return shintake;
  // }

  public void clearPDHStickyFaults() {
    pdh.clearStickyFaults();
  }


  

}
