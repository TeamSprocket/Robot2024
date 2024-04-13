// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class Climb extends Command {

  Elevator elevator;
  Timer timer;
  /** Creates a new Climb. */
  public Climb(Elevator elevator) {
    this.elevator = elevator;
    timer.reset();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setState(ElevatorStates.CLIMB_UP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevator.setState(ElevatorStates.CLIMB_UP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (elevator.isAtTargetGoal() && timer.get() > 1.5) {
      elevator.setState(ElevatorStates.CLIMB_DOWN);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
