
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerSubwooferShootTimed extends Command { // EXACT SAME AS ScoreSpeakerSubwoofer.java
  Timer overallTimer = new Timer();
  double duration;

  Shooter shooter;
  Intake intake;
  Timer waitTimer = new Timer();
  Timer scoreTimer = new Timer();

  public ScoreSpeakerSubwooferShootTimed(Shooter shooter, Intake intake, double duration) {
    this.shooter = shooter;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    overallTimer.reset();
    overallTimer.start();


    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);

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
    } else {
      waitTimer.reset();
      waitTimer.stop();
    }

    // Start timed scoring sequence
    if (waitTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      // intake.setState(IntakeStates.SCORE_SPEAKER); // We don't need this yet
      scoreTimer.start();   
    }

    if (scoreTimer.get() > Constants.Superstructure.kScoreSpeakerPivotTimeToleranceSec) {
      shooter.setState(ShooterStates.SCORE_SPEAKER_SUBWOOFER);
      intake.setState(IntakeStates.SCORE_SPEAKER);
    }

    SmartDashboard.putNumber("Score Timer", scoreTimer.get());
    SmartDashboard.putBoolean("Score Timer Over Threshold", scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec);


  }

  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
  }

  @Override
  public boolean isFinished() {
    return scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec || overallTimer.get() > duration;
  }
}
