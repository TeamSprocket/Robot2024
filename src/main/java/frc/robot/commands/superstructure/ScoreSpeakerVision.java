// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class ScoreSpeakerVision extends Command {
  Intake intake;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Vision vision;
  Timer timer;

  /** Creates a new ScoreSpeakerVision. */
  public ScoreSpeakerVision(Intake intake, Shooter shooter, ShooterPivot shooterPivot, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle = vision.shooterPivotAngle();
    shooterPivot.setTargetPivotAngle(angle);

    shooterPivot.setState(ShooterPivotStates.SPEAKER);
    shooter.setState(ShooterStates.SPINUP);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setState(ShooterStates.SPINUP);
    shooterPivot.setState(ShooterPivotStates.SPEAKER);

    if (timer.get() > 0.5) {
      intake.setState(IntakeStates.SCORE_SPEAKER);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
