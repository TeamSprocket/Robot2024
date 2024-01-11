// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

public class ZeroGyro extends InstantCommand {
  SwerveDrive swerveDrive;
  public ZeroGyro(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.swerveDrive.zeroGyro();
    this.swerveDrive.setTargetHeadingRad(swerveDrive.getHeading());
  }
}
