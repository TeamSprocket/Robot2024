// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class Climb extends Command {

  Elevator elevator;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Intake intake;

  Timer timer;
  /** Creates a new Climb. */
  public Climb(Elevator elevator, Shooter shooter, ShooterPivot shooterPivot, Intake intake) {
    this.elevator = elevator;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.intake = intake;

    timer.reset();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
    intake.setState(IntakeStates.STOWED);

    elevator.setState(ElevatorStates.CLIMB_UP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Continuously prevent state updates 
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
    intake.setState(IntakeStates.STOWED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setState(ElevatorStates.CLIMB_DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
