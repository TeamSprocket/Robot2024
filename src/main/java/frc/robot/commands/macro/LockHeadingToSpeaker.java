// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.OLDSwerveDrive;

public class LockHeadingToSpeaker extends Command {

  OLDSwerveDrive swerveDrive;
  public LockHeadingToSpeaker(OLDSwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.robotState = RobotState.TELEOP_LOCK_TURN_TO_SPEAKER;
    swerveDrive.updateLastOffsets();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Constants.robotState = RobotState.TELEOP_LOCK_TURN_TO_SPEAKER;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.robotState = RobotState.TELEOP;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
