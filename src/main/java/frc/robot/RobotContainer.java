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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.EjectNote;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.controls.Controller;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.Telemetry;

public class RobotContainer {

  // public final static Controller driver = new Controller(0);
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  // public final static Controller operator = new Controller(1);
  private final CommandXboxController operator = new CommandXboxController(1);

  PowerDistribution pdh = new PowerDistribution();

  private final TunerConstants tunerConst = new TunerConstants();
  private final CommandSwerveDrivetrain drivetrain = tunerConst.DriveTrain; // My drivetrain
  private final Telemetry logger = new Telemetry(Constants.Drivetrain.MaxSpeed);
  
  private final ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getLeftY(), () -> drivetrain.getTranslation3d());
  private final Shooter shooter = new Shooter(() -> drivetrain.getPose().getTranslation(), () -> operator.getRightTriggerAxis(), () -> operator.getLeftTriggerAxis(), () -> drivetrain.getTranslation3d());
  private final Intake intake = new Intake();

  Superstructure superstructure = new Superstructure(shooterPivot, shooter, intake);

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

    new Trigger(operator.rightBumper()) // TEST
      .whileTrue(superstructure.setState(SSStates.SCORE))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.x())
      .whileTrue(superstructure.setState(SSStates.WAIT_SPEAKER_SUBWOOFER)) // skipped this step and went straight to running indexer
      // .whileTrue(new ConditionalCommand(superstructure.setState(SSStates.SCORE), superstructure.setState(SSStates.STOWED), () -> shooter.atGoalShooter()))
      // .whileTrue(superstructure.setState(SSStates.WAIT_SPEAKER_SUBWOOFER)
      //   .alongWith(new WaitCommand(0.3))
      //   .andThen(new ConditionalCommand(superstructure.setState(SSStates.SCORE), superstructure.setState(SSStates.STOWED), () -> shooter.atGoalShooter())))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.a())
      .whileTrue(superstructure.setState(SSStates.INTAKE))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.y())
      .whileTrue(superstructure.setState(SSStates.EJECT_NOTE))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.b())
      .whileTrue(superstructure.setState(SSStates.CROSSFIELD))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    // operator.rightBumper().whileTrue(superstructure.setState(SSStates.SCORE)); //
    // operator.x().whileTrue(superstructure.setState(SSStates.WAIT_SPEAKER_SUBWOOFER));
    // operator.a().whileTrue(superstructure.setState(SSStates.INTAKE));
    // operator.button(RobotMap.Controller.MENU_BUTTON).whileTrue(superstructure.setState(SSStates.EJECT_NOTE)); // View button
    // operator.button(RobotMap.Controller.VIEW_BUTTON).whileTrue(superstructure.setState(SSStates.CROSSFIELD)); // Menu button
  }
  
  public ShooterPivot getShooterPivot() {
    return shooterPivot;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public void clearPDHStickyFaults() {
    pdh.clearStickyFaults();
  }

  public void zeroSuperstructurePositions() {
    shooter.zeroPosition();
    intake.zeroPosition();
    shooterPivot.zeroPosition();
  }
}
