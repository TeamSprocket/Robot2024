// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OLDSwerveDrive;
import frc.util.Util;

public class Backup extends Command {
  /** Creates a new MidlineLeft. */
  OLDSwerveDrive swerveDrive;
  double desiredMetersX;
  double desiredMetersY;

  PIDController pidController = new PIDController(0, 0, 0);

  public Backup(OLDSwerveDrive swerveDrive, double desiredMetersX, double desiredMetersY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    this.desiredMetersX = desiredMetersX;
    this.desiredMetersY = desiredMetersY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double outputX = Util.minmax(pidController.calculate(swerveDrive.getPose().getX(), desiredMetersX), -1, 1);
    double outputY = Util.minmax(pidController.calculate(swerveDrive.getPose().getY(), desiredMetersY), -1, 1);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(outputX, outputY, 0, new Rotation2d(swerveDrive.getHeading()));
    SwerveModuleState[] moduleStates = Constants.OldDrivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.OldDrivetrain.kMaxSpeed);
    swerveDrive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
