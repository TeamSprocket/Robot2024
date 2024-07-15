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

public class ShootNote extends Command {

  Shooter shooter;
  Intake intake;
  ShooterPivot shooterPivot;
  Timer waitTimer = new Timer();
  Timer scoreTimer = new Timer();
  Timer timer = new Timer();

  public ShootNote(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    // intake.setState(IntakeStates.SCORE_SPEAKER);

    waitTimer.reset();
    waitTimer.stop();

    scoreTimer.reset();
    scoreTimer.stop();

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    
    if (shooter.atGoalShooter() || timer.get() > 3.0) { // At speed
      waitTimer.start();
    } 

    // Start timed scoring sequence
    if (waitTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      scoreTimer.start();   
      shooter.setIndexerSpeedScoreSpeaker();
      // System.out.println("Shoot Note\nShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot NoteShoot Note");
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
