// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.DoNothing;
import frc.robot.auto.OneNoteNoTaxi;
import frc.robot.commands.EjectNote;
import frc.robot.commands.auto.IntakeNoteManualTimed;
import frc.robot.commands.auto.PreloadtoMidlineBlue;
import frc.robot.commands.auto.PreloadtoMidlineRed;
// import frc.robot.commands.auto.OneNoteAuton;
import frc.robot.commands.auto.ScoreSpeakerSubwooferShootTimed;
import frc.robot.commands.auto.ScoreSpeakerSubwooferSpinupTimed;
import frc.robot.commands.instant.*;
import frc.robot.commands.macro.LockHeadingToSpeaker;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.controls.Controller;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final static Controller driver = new Controller(0);
  public final static Controller operator = new Controller(1);

  PowerDistribution pdh = new PowerDistribution();

  Vision limelight = new Vision();
  SwerveDrive swerveDrive = new SwerveDrive(limelight);
  // Elevator elevator = new Elevator(() -> operator.getController().getRightY());
  ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getController().getLeftY(), () -> swerveDrive.getTranslation3d());
  Shooter shooter = new Shooter(() -> swerveDrive.getPose().getTranslation(), () -> operator.getController().getRightTriggerAxis(), () -> operator.getController().getLeftTriggerAxis(), () -> swerveDrive.getTranslation3d());
  Intake intake = new Intake();

  Pose2d robotPose = new Pose2d();

  

  // Superstructure superstructure = new Superstructure(elevator, shooterPivot, shooter, intake);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }

  public void initAutons() {
    autonChooser.setDefaultOption("Do Nothing", new DoNothing());
    
    // autonChooser.addOption("Figure Eight Test", new PathPlannerAuto("FigEightTestAuton"));

    // autonChooser.addOption("ANY Taxi", new PathPlannerAuto("ANY Taxi"));

    // autonChooser.addOption("Preload Early", new PathPlannerAuto("Preload Early"));
    // autonChooser.addOption("Preload Late", new PathPlannerAuto("Preload Late"));

    // path planner

    // autonChooser.addOption("Preload + go to midline BLUE", new PathPlannerAuto("Preload + Midline BLUE")); // test if this works with alliance switching
    // autonChooser.addOption("Preload + go to midline RED", new PathPlannerAuto("Preload + Midline RED"));

    // by encoder ticks

    // autonChooser.addOption("PreloadMidlineBlue", new PreloadtoMidlineBlue(swerveDrive, intake, shooterPivot, shooter));
    // autonChooser.addOption("PreloadMidlineRed", new PreloadtoMidlineRed(swerveDrive, intake, shooterPivot, shooter));

    // autonChooser.addOption("B2 2Note", new PathPlannerAuto("B2 2Note"));

    // autonChooser.addOption("", getAutonomousCommand());
    // autonChooser.addOption("Center 1 + 0 to Midline", new PathPlannerAuto("Center 1 + 0 to Midline"));
    // autonChooser.addOption("Disrupt Left", new PathPlannerAuto("Disrupt Left"));
    // autonChooser.addOption("Disrupt Right", new PathPlannerAuto("Disrupt Right"));
    // autonChooser.addOption("Fig Eight Test", new PathPlannerAuto("Fig Eight"));
    // autonChooser.addOption("Left 1 + 0 to Midline", new PathPlannerAuto("Left 1 + 0 to Midline"));
    // autonChooser.addOption("Left 1 + 1 to Midline", new PathPlannerAuto("Left 1 + 1 to Midline"));
    // autonChooser.addOption("Left 1 + 2 to Midline", new PathPlannerAuto("Left 1 + 2 to Midline"));
    // autonChooser.addOption("Middle 1 + 3", new PathPlannerAuto("Middle 1 + 3"));
    // autonChooser.addOption("Right 1 + 0 to Midline", new PathPlannerAuto("Right 1 + 0 to Midline"));
    // autonChooser.addOption("Right 1 + 1 to Midline", new PathPlannerAuto("Right 1 + 1 to Midline"));
    // autonChooser.addOption("Right 1 + 2 to Midline", new PathPlannerAuto("Right 1 + 2 to Midline"));

    // autonChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public void initNamedCommands() {
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, shooter, shooterPivot));
    NamedCommands.registerCommand("SpinupSubwoofer", new ScoreSpeakerSubwooferSpinupTimed(shooter, shooterPivot, 3.0));
    NamedCommands.registerCommand("ShootNote", new ShootNote(shooterPivot, shooter, intake));
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
    driver.getController().leftBumper().whileTrue(new LockHeadingToSpeaker(swerveDrive));
    
    
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
    // operator.getController().leftBumper().whileTrue(new ScoreAmp(elevator, shooterPivot, shooter, intake));
    operator.getController().rightBumper().whileTrue(new ShootNote(shooterPivot, shooter, intake));
    // operator.getController().rightBumper().whileTrue(new ScoreSpeaker(shooterPivot, shooter, intake));

    // operator.getController().y().whileTrue(new WaitAmp(elevator, shooterPivot, intake));
    operator.getController().x().whileTrue(new ScoreSpeakerSubwooferSpinup(shooter, shooterPivot));

    // operator.getController().b().whileTrue(new ScoreSpeakerPodiumSpinup(shooterPivot, shooter, swerveDrive));
    operator.getController().a().whileTrue(new IntakeNote(intake, shooter, shooterPivot));
    // operator.getController().b().whileTrue(new ScoreSpeakerShoot(shooterPivot, shooter, swerveDrive));
    // operator.getController().b().whileTrue(new ScoreSpeakerAmpZone(shooterPivot, shooter, swerveDrive));
    // operator.getController().b().whileTrue(new ScoreSpeakerPodiumShoot(shooterPivot, shooter, swerveDrive));
    // operator.getController().a().onTrue(new IntakeNote(intake, shooter, shooterPivot));
    // operator.getController().b().onTrue(new CancelIntake(intake, shooter, shooterPivot));

    operator.getController().button(RobotMap.Controller.MENU_BUTTON).whileTrue(new EjectNote(intake, shooter, shooterPivot)); // View button
    operator.getController().button(RobotMap.Controller.VIEW_BUTTON).whileTrue(new ShootCrossfieldSpinup(shooterPivot, shooter, intake)); // Menu button

    // operator.getController().leftTrigger(Constants.Controller.kClimbTriggerAxisPercent).onTrue(new ClimbUp(elevator, shooter, shooterPivot, intake));
    // operator.getController().rightTrigger(Constants.Controller.kClimbTriggerAxisPercent).onTrue(new ClimbDown(elevator, intake));


    // operator.getController().button(RobotMap.Controller.LEFT_STICK_BUTTON).onTrue(new ReIndexNote(shooter, shooterPivot));

    // elevator.setDefaultCommand(new ElevatorManual(elevator, () -> operator.getController().getLeftY()));

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

  public Intake getIntake() {
    return intake;
  }

  public void clearPDHStickyFaults() {
    pdh.clearStickyFaults();
  }

  public void zeroSuperstructurePositions() {
    shooter.zeroPosition();
    intake.zeroPosition();
    shooterPivot.zeroPosition();
    // elevator.zeroPosition();
    
  }
  
  public Pose2d getPose2d() {
    if (limelight.hasTargets()) {
      robotPose = limelight.getPose2d();
    }
    else {
      swerveDrive.resetPose(robotPose);
      robotPose = swerveDrive.odometry.getPoseMeters();
    }
    return robotPose;
  }
  

  


  

}
