// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator.ClimbStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;

public class ClimbDown extends Command {
  /** Creates a new ClimbDown. */
  Elevator elevator;
  Intake intake;
  Timer timer = new Timer();

  public ClimbDown(Elevator elevator, Intake intake) {
    this.elevator = elevator;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setState(ElevatorStates.FIND_BOTTOM);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeStates.STOWED);
    elevator.setState(ElevatorStates.NONE);
    // CommandScheduler.getInstance().cancelAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("COMMAND HAS ENDEDCOMMAND HAS ENDEDCOMMAND HAS ENDEDCOMMAND HAS ENDEDCOMMAND HAS ENDEDCOMMAND HAS ENDED");
    // return elevator.climbHasHooked();
    return hasFinishedClimbing() || timer.get() > 3.0;
  }


  public boolean hasFinishedClimbing() {
    return elevator.climbHasHitBottom() && timer.get() > 0.5;
  }
}
