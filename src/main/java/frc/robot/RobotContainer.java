// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.swerve.Telemetry;
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);

  PowerDistribution pdh = new PowerDistribution();
  private final TunerConstants tunerConst = new TunerConstants();
  private final CommandSwerveDrivetrain drivetrain = tunerConst.DriveTrain; // My drivetrain  
  private final ShooterPivot shooterPivot = new ShooterPivot(() -> operator.getLeftY(), () -> drivetrain.getTranslation3d());
  private final Shooter shooter = new Shooter(() -> drivetrain.getPose().getTranslation(), () -> operator.getRightTriggerAxis(), () -> operator.getLeftTriggerAxis(), () -> drivetrain.getTranslation3d());
  private final Intake intake = new Intake();
  private final Vision vision = new Vision(drivetrain);
  Superstructure superstructure = new Superstructure(shooterPivot, shooter, intake);

  // ------- Swerve Generated -------

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drivetrain.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drivetrain.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.ApplyChassisSpeeds headingLock = new SwerveRequest.ApplyChassisSpeeds()
    .withSteerRequestType(SteerRequestType.MotionMagic);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }
  
 public void initAutons() {

    // ------ path planner ------

    // autonChooser.addOption("ANY Taxi", new PathPlannerAuto("ANY Taxi"));
    autonChooser.setDefaultOption("Fig Eight Test", new PathPlannerAuto("Fig Eight"));

    // ------- by encoder ticks -------

    // autonChooser.addOption("PreloadMidlineBlue", new PreloadtoMidlineBlue(swerveDrive, intake, shooterPivot, shooter));
    // autonChooser.addOption("PreloadMidlineRed", new PreloadtoMidlineRed(swerveDrive, intake, shooterPivot, shooter));
    
    autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

   public void initNamedCommands() {
    NamedCommands.registerCommand("IntakeNote", new SequentialCommandGroup(superstructure.setState(SSStates.INTAKE)
                                                                                .andThen(new WaitUntilCommand(() -> shooter.beamBroken()))
                                                                                .andThen(superstructure.setState(SSStates.STOWED))));

    // NamedCommands.registerCommand("SpinupSubwoofer", new SequentialCommandGroup(new WaitCommand(0.2)
                                                                                    //  .andThen(alignSwerveCommand().withTimeout(1))
                                                                                    //  .andThen(new WaitUntilCommand(() -> vision.isAligned()).withTimeout(1))
                                                                                    //  .andThen(superstructure.setState(SSStates.WAIT_SPEAKER_PODIUM))));
    
    // NamedCommands.registerCommand("SpinupSubwoofer", new ScoreSpeakerSubwooferSpinupTimed(shooter, shooterPivot, 3.0));
    // NamedCommands.registerCommand("ShootNote", new SequentialCommandGroup(new WaitCommand(0.5) // wait for intake to move in
    //                                                                            .andThen(new InstantCommand(()-> shooter.setIndexerSpeedScoreSpeaker())) // spit out note for 0.2
    //                                                                            .andThen(new WaitCommand(0.2))
    //                                                                            .andThen(superstructure.setState(SSStates.STOWED))));

    // NamedCommands.registerCommand("SetStowed", superstructure.setState(SSStates.STOWED));

    // NamedCommands.registerCommand("ShootCrossfield", superstructure.setState(SSStates.CROSSFIELD));

    // NamedCommands.registerCommand("EjectNote", superstructure.setState(SSStates.EJECT_NOTE));

    // NamedCommands.registerCommand("ShootNote", new ShootNote(shooterPivot, shooter, intake));
    // NamedCommands.registerCommand("ScoreSpeakerPodiumSpinup", new ScoreSpeakerPodiumSpinup(shooterPivot, shooter, CommandSwerveDrivetrain));
    // NamedCommands.registerCommand("ScoreSpeakerSubwooferShoot", new ScoreSpeakerSubwooferShootTimed(shooter, intake, shooterPivot));
    // NamedCommands.registerCommand("PrintHello", new PrintHello());dadad

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
    driver.y().whileTrue(alignSwerveCommand());
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    // --------------------=operator=--------------------

    // ----- rumble controllers -----
    new Trigger(() -> shooter.beamBroken()) // TODO: controllers are rumbling too much (might add something to make it stop)
      .onTrue(rumbleControllers());

    new Trigger(operator.rightBumper())
      .whileTrue(new WaitCommand(0.5) // wait for intake to move in
        .andThen(new InstantCommand(()-> shooter.setIndexerSpeedScoreSpeaker())) // spit out note for 0.2
        .andThen(new WaitCommand(0.2))
        .andThen(superstructure.setState(SSStates.STOWED))
      );
      
    new Trigger(operator.x())
      .whileTrue(superstructure.setState(SSStates.WAIT_SPEAKER_SUBWOOFER))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.button(8))
      .whileTrue(alignSwerveCommand().withTimeout(1)
        .andThen(new WaitUntilCommand(() -> vision.isAligned()).withTimeout(1))
        .andThen(superstructure.setState(SSStates.WAIT_SPEAKER_PODIUM)));
    new Trigger(operator.button(8))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.y())
      .whileTrue(superstructure.setState(SSStates.EJECT_NOTE));
      // .andThen(superstructure.setState(SSStates.STOWED)));
    new Trigger(operator.y())
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.b())
      .whileTrue(superstructure.setState(SSStates.CROSSFIELD))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.a())
      .whileTrue(superstructure.setState(SSStates.INTAKE)
        .andThen(new WaitUntilCommand(() -> shooter.beamBroken()))
        .andThen(superstructure.setState(SSStates.STOWED)));

    new Trigger(operator.a())
      .onFalse(superstructure.setState(SSStates.INTAKE_BACK)
        .andThen(new WaitCommand(0.1))
        .andThen(superstructure.setState(SSStates.STOWED)));
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

  public Command rumbleControllers() {
    return Commands.runOnce(() ->
      CommandScheduler.getInstance().schedule(
        Commands.sequence(
          Commands.waitSeconds(0.5),
          Commands.runOnce(() -> {
              operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
              driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
          }),
          Commands.waitSeconds(0.5),
          Commands.runOnce(() -> {
              operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
              driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
          })
        )
      )
    );
  }

  public Command alignSwerveCommand() {
    return drivetrain.applyRequest(() -> headingLock.withSpeeds(vision.getHeadingLockSpeed()));
  }
}