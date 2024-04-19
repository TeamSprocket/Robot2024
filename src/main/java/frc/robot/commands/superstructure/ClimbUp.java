// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class ClimbUp extends Command {

  Elevator elevator;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Intake intake;
  boolean hasZeroed;

  Timer timer = new Timer();
  /** Creates a new Climb. */
  public ClimbUp(Elevator elevator, Shooter shooter, ShooterPivot shooterPivot, Intake intake) {
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
    hasZeroed = false;
    
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
    intake.setState(IntakeStates.CLIMB);
    elevator.setState(ElevatorStates.STOWED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasZeroed) {
      elevator.setState(ElevatorStates.ZEROING);
    } 
    
    if (!hasZeroed && elevator.elevatorHitBottom()) {
      elevator.zeroPosition();
      hasZeroed = true;
    }

    if (hasZeroed && elevator.getState() != ElevatorStates.CLIMB_UP) {
      elevator.setState(ElevatorStates.CLIMB_UP);
    }

    // Continuously prevent state updates 
    // shooter.setState(ShooterStates.STANDBY);
    // shooterPivot.setState(ShooterPivotStates.STOWED);
    // intake.setState(IntakeStates.CLIMB);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new ClimbDown(elevator, intake));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
