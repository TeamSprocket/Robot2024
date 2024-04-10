
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerSubwoofer extends Command {

  Shooter shooter;
  Timer waitTimer = new Timer();
  Timer scoreTimer = new Timer();

  public ScoreSpeakerSubwoofer(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);

    waitTimer.reset();
    waitTimer.stop();

    scoreTimer.reset();
    scoreTimer.stop();
  }

  @Override
  public void execute() {
    if (shooter.atGoalShooter()) { // At speed
      waitTimer.start();
    } 
    // else {
    //   waitTimer.reset();
    //   waitTimer.stop();
    // }

    // Start timed scoring sequence
    if (waitTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      shooter.setState(ShooterStates.SCORE_SPEAKER_SUBWOOFER);
      scoreTimer.start();
    }

  }

  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterStates.STANDBY);
  }

  @Override
  public boolean isFinished() {
    return scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec;
  }
}
