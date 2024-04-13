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

public class ScoreSpeaker extends Command {

  Shooter shooter;
  Intake intake;
  ShooterPivot shooterPivot;
  Timer intakeTimer = new Timer();
  Timer waitTimer = new Timer();
  Timer scoreTimer = new Timer();

  public ScoreSpeaker(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.setState(IntakeStates.SCORE_SPEAKER);
    shooter.setState(ShooterStates.SPINUP);

    waitTimer.reset();
    waitTimer.stop();

    scoreTimer.reset();
    scoreTimer.stop();

    intakeTimer.reset();
    intakeTimer.start();
  }

  @Override
  public void execute() {

    if (intakeTimer.get() > 0.1) {
      shooterPivot.setState(ShooterPivotStates.SPEAKER);
    }
    
    if (shooter.atGoalShooter() && shooterPivot.atGoal()) { // At speed
      waitTimer.start();
    } 

    // Start timed scoring sequence
    if (waitTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      scoreTimer.start();   
      shooter.setState(ShooterStates.SCORE_SPEAKER); 
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
    return scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec + Constants.Superstructure.kWaitSpeakerTimeToleranceSec;
  }
}
