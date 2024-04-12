package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.SwerveDrive;

public class ScoreSpeakerPodium extends Command {

  Shooter shooter;
  ShooterPivot shooterPivot;
  SwerveDrive swerveDrive;
  Timer waitTimer = new Timer();
  Timer scoreTimer = new Timer();

  public ScoreSpeakerPodium(ShooterPivot shooterPivot, Shooter shooter, SwerveDrive swerveDrive) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    Constants.robotState = RobotState.TELEOP_LOCK_TURN_TO_SPEAKER;
    swerveDrive.updateLastOffsets();

    shooter.setState(ShooterStates.SPINUP_PODIUM);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);

    waitTimer.reset();
    waitTimer.stop();

    scoreTimer.reset();
    scoreTimer.stop();
  }

  @Override
  public void execute() {
    
    if (shooter.atGoalShooter() && swerveDrive.isAlignedWithTarget()) { // At speed
      waitTimer.start();
    } 

    // Start timed scoring sequence
    if (waitTimer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      scoreTimer.start();   
      shooter.setState(ShooterStates.SCORE_SPEAKER_PODIUM); 
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec + Constants.Superstructure.kWaitSpeakerTimeToleranceSec;
  }
}
