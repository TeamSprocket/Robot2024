// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.instant.SwitchTargetHeadingDirection;
import frc.robot.commands.instant.ZeroGyro;
import frc.robot.commands.macro.ZeroWheelsTEST;
import frc.robot.commands.persistent.DriveTeleop;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.Directions;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);

  SwerveDrive swerveDrive = new SwerveDrive();

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();


  public RobotContainer() {
    configureBindings();
    initAutons();
  }

  public void initAutons() {

    Command figureEightTestAuton = new PathPlannerAuto("FigEightTestAuton");
    Command turn90and1MTestAuton = new PathPlannerAuto("Turn 90 and 1M Test Auton");

    autonChooser.addOption("Figure Eight Test", figureEightTestAuton);
    autonChooser.addOption("Turn 90 and 1M Test", turn90and1MTestAuton);
    autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Chooser", autonChooser);
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
			() -> driver.getRightX()));
    // driver.x().onTrue(new ZeroWheelsTEST(swerveDrive));
    driver.rightBumper().onTrue(new ZeroGyro(swerveDrive));
    driver.button(RobotMap.Controller.Y).onTrue(new SwitchTargetHeadingDirection(swerveDrive, Directions.FORWARD));
    driver.button(RobotMap.Controller.X).onTrue(new SwitchTargetHeadingDirection(swerveDrive, Directions.LEFT));
    driver.button(RobotMap.Controller.B).onTrue(new SwitchTargetHeadingDirection(swerveDrive, Directions.RIGHT));
    driver.button(RobotMap.Controller.A).onTrue(new SwitchTargetHeadingDirection(swerveDrive, Directions.BACK));
  }
  
  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

}
