// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.EjectNote;
import frc.robot.commands.auto.PreloadtoMidlineBlue;
import frc.robot.commands.auto.PreloadtoMidlineRed;
import frc.robot.commands.macro.LockHeadingToSpeaker;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.controls.Controller;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.Telemetry;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  public final static Controller operator = new Controller(1);

  PowerDistribution pdh = new PowerDistribution();

  private final TunerConstants tunerConstants = new TunerConstants();
  private final CommandSwerveDrivetrain swerveDrive = tunerConstants.DriveTrain; // My drivetrain

  private final Telemetry logger = new Telemetry(Constants.Drivetrain.MaxSpeed);
  private final Elevator elevator = new Elevator(() -> operator.getController().getRightY());
  private final ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getController().getLeftY(), () -> swerveDrive.getTranslation3d());
  private final Shooter shooter = new Shooter(() -> swerveDrive.getPose().getTranslation(), () -> operator.getController().getRightTriggerAxis(), () -> operator.getController().getLeftTriggerAxis(), () -> swerveDrive.getTranslation3d());
  private final Intake intake = new Intake();

  // ------- Swerve Generated -------

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drivetrain.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drivetrain.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initAutons();
  }

  public void initAutons() {

    // ------- by encoder ticks -------

    autonChooser.addOption("PreloadMidlineBlue", new PreloadtoMidlineBlue(swerveDrive, intake, shooterPivot, shooter));
    autonChooser.addOption("PreloadMidlineRed", new PreloadtoMidlineRed(swerveDrive, intake, shooterPivot, shooter));
    
    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void configureBindings() {

    // --------------------=Driver=--------------------

    swerveDrive.setDefaultCommand( // Drivetrain will execute this command periodically
      swerveDrive.applyRequest(() -> drive.withVelocityX(driver.getLeftY() * Constants.Drivetrain.MaxSpeed * 0.5) // Drive forward with negative Y (forward)
        .withVelocityY(driver.getLeftX() * Constants.Drivetrain.MaxSpeed * 0.5) // Drive left with negative X (left) //test
        .withRotationalRate(swerveDrive.getTeleopTSpeed(()-> -driver.getRightX() * Constants.Drivetrain.MaxAngularRate)) // Drive counterclockwise with negative X (left)
        // !! ^
      ));

    driver.a().whileTrue(swerveDrive.applyRequest(() -> brake));
    driver.b().whileTrue(swerveDrive
      .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(swerveDrive.runOnce(() -> swerveDrive.seedFieldRelative()));

    if (Utils.isSimulation()) {
      swerveDrive.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    swerveDrive.registerTelemetry(logger::telemeterize);
    driver.leftBumper().whileTrue(new LockHeadingToSpeaker(swerveDrive));

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
