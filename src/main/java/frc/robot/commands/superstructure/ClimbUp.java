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
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ClimbStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class ClimbUp extends Command {

  Elevator elevator;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Intake intake;

  boolean hasFoundCeiling;

  Timer timer = new Timer();
  /** Creates a new Climb. */
  public ClimbUp(Elevator elevator, Shooter shooter, ShooterPivot shooterPivot, Intake intake) {
    this.elevator = elevator;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
    intake.setState(IntakeStates.CLIMB);

    elevator.setState(ElevatorStates.FIND_TOP);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Continuously prevent state updates 
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
    intake.setState(IntakeStates.CLIMB);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // CommandScheduler.getInstance().schedule(new ClimbDown(elevator));
    elevator.setTopHeight(elevator.getHeight());
    elevator.setState(ElevatorStates.CLIMB_HOLD_TOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.climbHasHitTop() || timer.get() > 4.0;
  }
}
