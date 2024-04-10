package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class ShootCrossfield extends Command {

  Shooter shooter;
  Intake intake; // check if intake is necessary
  ShooterPivot shooterPivot;
  Timer timer = new Timer();
  Timer scoreTimer = new Timer();
  Boolean isShooting;

  public ShootCrossfield(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;
    
    this.isShooting = false;
  }

  @Override
  public void initialize() {
    intake.setState(IntakeStates.SCORE_SPEAKER);
    shooterPivot.setState(ShooterPivotStates.CROSSFIELD);
    shooter.setState(ShooterStates.SPINUP_CROSSFIELD);

    timer.reset();
    timer.start();

    scoreTimer.reset();
    scoreTimer.stop();
  }

  @Override
  public void execute() {
    if (shooter.atGoalShooter() && timer.get() > 1.0) {
      scoreTimer.start();
    } 
    if (!shooter.atGoalShooter() && !isShooting) {
      scoreTimer.reset();
      scoreTimer.stop();
    }

    if (scoreTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      isShooting = true;
      shooter.setState(ShooterStates.SHOOT_CROSSFIELD);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootTimeToleranceSec + Constants.Superstructure.kWaitSpeakerTimeToleranceSec;
  }
}
