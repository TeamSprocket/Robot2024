// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class EjectNote extends Command {
  /** Creates a new EjectIntake. */
  Intake intake;
  Shooter shooter;
  ShooterPivot pivot;
  Timer waitTimer = new Timer();

  public EjectNote(Intake intake, Shooter shooter, ShooterPivot pivot) {
    this.intake = intake;
    this.shooter = shooter;
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     waitTimer.reset();
      waitTimer.start();

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (waitTimer.get() < 2.0) {
      intake.setState(IntakeStates.EJECT_NOTE);
      if (waitTimer.get() > 0.5) {
        pivot.setState(ShooterPivotStates.EJECT_NOTE);
      }
      if (waitTimer.get() > 1.5){
        shooter.setState(ShooterStates.EJECT_NOTE);
      }
    } else {
      pivot.setState(ShooterPivotStates.STOWED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
    pivot.setState(ShooterPivotStates.STOWED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return waitTimer.get() > 2.5;
  }
}
