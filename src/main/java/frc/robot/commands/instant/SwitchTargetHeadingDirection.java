// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.Directions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwitchTargetHeadingDirection extends InstantCommand {
  SwerveDrive swerveDrive;
  Directions direction;
  public SwitchTargetHeadingDirection(SwerveDrive swerveDrive, Directions direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    this.direction = direction;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.swerveDrive.switchDirection(direction);
  }
}
