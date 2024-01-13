// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class ZeroWheelsTEST extends CommandBase {
  /** Creates a new ZeroWheelsTEST. */
  SwerveDrive swerveDrive;
  public ZeroWheelsTEST(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RESET THE WHEELS (DENIELLE IS HOMOPHOBIC)");
    System.out.println("OFFSET FL: " + Constants.Drivetrain.kCANCoderOffsetFrontLeft);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.resetModulesToAbsolute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
