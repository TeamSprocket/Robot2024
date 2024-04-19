// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class CancelIntake extends Command {
  /** Creates a new CancelIntake. */
  Intake intake;
  Shooter shooter;
  ShooterPivot shooterPivot;

  public CancelIntake(Intake intake, Shooter shooter, ShooterPivot shooterPivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.Intake.RunIntake = false;
    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);    
    shooterPivot.setState(ShooterPivotStates.STOWED);

    // CommandScheduler.getInstance().cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
