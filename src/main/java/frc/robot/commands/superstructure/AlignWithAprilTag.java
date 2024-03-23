// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class AlignWithAprilTag extends Command {
  /** Creates a new AlignWithAprilTag. */
  SwerveDrive swerveDrive;

  public AlignWithAprilTag(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
     this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.lockHeading();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(swerveDrive.getTspeed()) < 0.01) {
      return true;
    } else {
      return false;
    }
  }
}
