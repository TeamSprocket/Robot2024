
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerShoot extends Command { // EXACT SAME AS ScoreSpeakerSubwoofer.java

  Shooter shooter;
  Intake intake;
  ShooterPivot shooterPivot;
  Timer waitTimer = new Timer();
  Timer scoreTimer = new Timer();

  public ScoreSpeakerShoot(Shooter shooter, Intake intake, ShooterPivot shooterPivot) {
    this.shooter = shooter;
    this.intake = intake;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void initialize() {
    shooter.setState(ShooterStates.SPINUP);
    shooterPivot.setState(ShooterPivotStates.SPEAKER);

    waitTimer.reset();
    waitTimer.stop();

    scoreTimer.reset();
    scoreTimer.stop();

    // System.out.println("RUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\nRUNNING SHOOTER\n");
  }

  @Override
  public void execute() {
    if (shooter.atGoalShooter()) { // At speed
      waitTimer.start();
    } 
    // else { // caused indexer to stutter
    //   waitTimer.reset();
    //   waitTimer.stop();
    // }

    // Start timed scoring sequence
    if (waitTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      // intake.setState(IntakeStates.SCORE_SPEAKER); // We don't need this yet
      scoreTimer.start();   
      shooter.setState(ShooterStates.SCORE_SPEAKER); 
      intake.setState(IntakeStates.SCORE_SPEAKER_SUBWOOFER);
    }

    // if (scoreTimer.get() > Constants.Superstructure.kScoreSpeakerPivotTimeToleranceSec) {
      // shooter.setState(ShooterStates.SCORE_SPEAKER_SUBWOOFER); 
    // } 
    

    SmartDashboard.putNumber("Score Timer", scoreTimer.get());
    SmartDashboard.putBoolean("Score Timer Over Threshold", scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec);


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
