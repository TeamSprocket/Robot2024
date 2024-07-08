// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
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
import frc.robot.commands.EjectNote;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.controls.Controller;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.Telemetry;

public class RobotContainer {

  // public final static Controller driver = new Controller(0);
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  public final static Controller operator = new Controller(1);

  PowerDistribution pdh = new PowerDistribution();

  private final TunerConstants tunerConst = new TunerConstants();
  private final CommandSwerveDrivetrain drivetrain = tunerConst.DriveTrain; // My drivetrain
  private final Telemetry logger = new Telemetry(Constants.Drivetrain.MaxSpeed);
  
  // SwerveDrive swerveDrive = new SwerveDrive(limelight);
  private final Elevator elevator = new Elevator(() -> operator.getController().getRightY());
  private final ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getController().getLeftY(), () -> drivetrain.getTranslation3d());
  private final Shooter shooter = new Shooter(() -> drivetrain.getPose().getTranslation(), () -> operator.getController().getRightTriggerAxis(), () -> operator.getController().getLeftTriggerAxis(), () -> drivetrain.getTranslation3d());
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

    // --------------------=operator=--------------------

    operator.getController().rightBumper().whileTrue(new ShootNote(shooterPivot, shooter, intake));
    operator.getController().x().whileTrue(new ScoreSpeakerSubwooferSpinup(shooter, shooterPivot));
    operator.getController().a().whileTrue(new IntakeNote(intake, shooter, shooterPivot));
    operator.getController().button(RobotMap.Controller.MENU_BUTTON).whileTrue(new EjectNote(intake, shooter, shooterPivot)); // View button
    operator.getController().button(RobotMap.Controller.VIEW_BUTTON).whileTrue(new ShootCrossfieldSpinup(shooterPivot, shooter, intake)); // Menu button
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
