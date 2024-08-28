// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.auto.*;
import frc.robot.commands.macro.LockHeadingToSpeaker;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.controls.Controller;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;

public class RobotContainer {

  // public final static Controller driver = new Controller(0);
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  public final static Controller operator = new Controller(1);

  PowerDistribution pdh = new PowerDistribution();

  private final TunerConstants tunerConst = new TunerConstants();
  private final CommandSwerveDrivetrain drivetrain = tunerConst.DriveTrain; // My drivetrain
  private final Telemetry logger = new Telemetry(Constants.Drivetrain.MaxSpeed);
  
  private SwerveModuleConstants FrontLeft;
  private SwerveModuleConstants FrontRight;
  private SwerveModuleConstants BackLeft;
  private SwerveModuleConstants BackRight;
  private SwerveDrivetrainConstants DrivetrainConstants;


  private final CommandSwerveDrivetrain CommandSwerveDrivetrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
      FrontRight, BackLeft, BackRight);
  private final Elevator elevator = new Elevator(() -> operator.getController().getRightY());
  private final ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getController().getLeftY());
  private final Shooter shooter = new Shooter(() -> operator.getController().getRightTriggerAxis(), () -> operator.getController().getLeftTriggerAxis());
  private final Intake intake = new Intake();

  // Superstructure superstructure = new Superstructure(elevator, shooterPivot, shooter, intake);

  // ------- Swerve Generated -------

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drivetrain.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drivetrain.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();


  
  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }

  public void initAutons() {

    // ------ path planner ------

    autonChooser.setDefaultOption("Do Nothing", new DoNothing());
    // autonChooser.addOption("Figure Eight Test", new PathPlannerAuto("FigEightTestAuton"));
    autonChooser.addOption("ANY Taxi", new PathPlannerAuto("ANY Taxi"));
    autonChooser.addOption("Preload Early", new PathPlannerAuto("Preload Early"));
    autonChooser.addOption("Preload Late", new PathPlannerAuto("Preload Late"));
    autonChooser.addOption("Preload + go to midline BLUE", new PathPlannerAuto("Preload + Midline BLUE")); // test if this works with alliance switching
    autonChooser.addOption("Preload + go to midline RED", new PathPlannerAuto("Preload + Midline RED"));
    autonChooser.addOption("Center 1 + 0 to Midline", new PathPlannerAuto("Center 1 + 0 to Midline"));
    autonChooser.addOption("Disrupt Left", new PathPlannerAuto("Disrupt Left"));
    autonChooser.addOption("Disrupt Right", new PathPlannerAuto("Disrupt Right"));
    autonChooser.addOption("Fig Eight Test", new PathPlannerAuto("Fig Eight"));
    autonChooser.addOption("Left 1 + 0 to Midline", new PathPlannerAuto("Left 1 + 0 to Midline"));
    autonChooser.addOption("Left 1 + 1 to Midline", new PathPlannerAuto("Left 1 + 1 to Midline"));
    autonChooser.addOption("Left 1 + 2 to Midline", new PathPlannerAuto("Left 1 + 2 to Midline"));
    autonChooser.addOption("Middle 1 + 3", new PathPlannerAuto("Middle 1 + 3"));
    autonChooser.addOption("Right 1 + 0 to Midline", new PathPlannerAuto("Right 1 + 0 to Midline"));
    autonChooser.addOption("Right 1 + 1 to Midline", new PathPlannerAuto("Right 1 + 1 to Midline"));
    autonChooser.addOption("Right 1 + 2 to Midline", new PathPlannerAuto("Right 1 + 2 to Midline"));
    

    // ------- by encoder ticks -------

    // autonChooser.addOption("PreloadMidlineBlue", new PreloadtoMidlineBlue(swerveDrive, intake, shooterPivot, shooter));
    // autonChooser.addOption("PreloadMidlineRed", new PreloadtoMidlineRed(swerveDrive, intake, shooterPivot, shooter));
    
    autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public void initNamedCommands() {
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, shooter, shooterPivot));
    NamedCommands.registerCommand("SpinupSubwoofer", new ScoreSpeakerSubwooferSpinupTimed(shooter, shooterPivot, 3.0));
    NamedCommands.registerCommand("ShootNote", new ShootNote(shooterPivot, shooter, intake));
    NamedCommands.registerCommand("ScoreSpeakerPodiumSpinup", new ScoreSpeakerPodiumSpinup(shooterPivot, shooter, CommandSwerveDrivetrain));
    NamedCommands.registerCommand("ScoreSpeakerSubwooferShoot", new ScoreSpeakerSubwooferShootTimed(shooter, intake, shooterPivot));
    // NamedCommands.registerCommand("PrintHello", new PrintHello());dadad

  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
    // return new OneNoteAuton(shooter, intake);
  }

  public void configureBindings() {

    // --------------------=Driver=--------------------
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive.withVelocityX(driver.getLeftY() * Constants.Drivetrain.MaxSpeed * 0.5) // Drive forward with negative Y (forward)
        .withVelocityY(driver.getLeftX() * Constants.Drivetrain.MaxSpeed * 0.5) // Drive left with negative X (left) //test
        .withRotationalRate(-driver.getRightX() * Constants.Drivetrain.MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain
      .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // swerveDrive.setDefaultCommand(new DriveTeleop(
    //     swerveDrive,
    //     () -> -driver.getController().getLeftX(),
    //     () -> driver.getController().getLeftY(),
    //     () -> -driver.getController().getRightX()));
    // driver.getController().rightBumper().onTrue(new InstantCommand(swerveDrive::zeroHeading)); // hehe it's faster :3
    // driver.getController().leftBumper().whileTrue(new LockHeadingToSpeaker(swerveDrive));
    
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

    operator.getController().rightBumper().whileTrue(new ShootNote(shooterPivot, shooter, intake));
    operator.getController().x().whileTrue(new ScoreSpeakerSubwooferSpinup(shooter, shooterPivot));
    // operator.getController().y().whileTrue(new ScoreSpeakerPodiumSpinup(shooterPivot, shooter, swerveDrive));
    operator.getController().a().whileTrue(new IntakeNote(intake, shooter, shooterPivot));
    operator.getController().b().onTrue(new CancelIntake(intake, shooter, shooterPivot));
    operator.getController().button(RobotMap.Controller.MENU_BUTTON).whileTrue(new EjectNote(intake, shooter, shooterPivot)); // View button
    operator.getController().button(RobotMap.Controller.VIEW_BUTTON).whileTrue(new ShootCrossfieldSpinup(shooterPivot, shooter, intake)); // Menu button

    // ------ elevator -------
    // operator.getController().leftBumper().whileTrue(new ScoreAmp(elevator, shooterPivot, shooter, intake));
    // operator.getController().y().whileTrue(new WaitAmp(elevator, shooterPivot, intake));
    // operator.getController().leftTrigger(Constants.Controller.kClimbTriggerAxisPercent).onTrue(new ClimbUp(elevator, shooter, shooterPivot, intake));
    // operator.getController().rightTrigger(Constants.Controller.kClimbTriggerAxisPercent).onTrue(new ClimbDown(elevator, intake));
    // elevator.setDefaultCommand(new ElevatorManual(elevator, () -> operator.getController().getLeftY()));
    
    // operator.getController().rightBumper().whileTrue(new ScoreSpeaker(shooterPivot, shooter, intake));
    // operator.getController().b().whileTrue(new ScoreSpeakerShoot(shooterPivot, shooter, swerveDrive));
    // operator.getController().b().whileTrue(new ScoreSpeakerAmpZone(shooterPivot, shooter, swerveDrive));
    // operator.getController().b().whileTrue(new ScoreSpeakerPodiumShoot(shooterPivot, shooter, swerveDrive));
    // operator.getController().button(RobotMap.Controller.LEFT_STICK_BUTTON).onTrue(new ReIndexNote(shooter, shooterPivot));

  }

  public void updateNoteRumbleListener() {
    // driver.updateNoteRumbleListener(shooter::beamBroken, shooter::getState); // not gonna deal with this rn :|
    operator.updateNoteRumbleListener(shooter::beamBroken, shooter::getState);
  }

  public void stopNoteRumbleListener() {
    // driver.stopNoteRumbleListener();
    operator.stopNoteRumbleListener();
  }
  
  public ShooterPivot getShooterPivot() {
    return shooterPivot;
  }

  public void clearPDHStickyFaults() {
    pdh.clearStickyFaults();
  }

  public void zeroSuperstructurePositions() {
    shooter.zeroPosition();
    intake.zeroPosition();
    shooterPivot.zeroPosition();
    elevator.zeroPosition();
    
  }
  

  

  


  

}