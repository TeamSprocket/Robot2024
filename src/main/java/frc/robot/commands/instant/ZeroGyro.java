// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OLDSwerveDrive;

public class ZeroGyro extends InstantCommand {
  OLDSwerveDrive swerveDrive;
  public ZeroGyro(OLDSwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.swerveDrive.zeroHeading();
    // this.swerveDrive.setTargetHeadingRad(swerveDrive.getHeading());
  }
}
