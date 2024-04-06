// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorUp extends Command {
  Elevator elevator;
  /** Creates a new ElevatorUp. */
  public ElevatorUp(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setState(ElevatorStates.AMP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setState(ElevatorStates.STOWED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
