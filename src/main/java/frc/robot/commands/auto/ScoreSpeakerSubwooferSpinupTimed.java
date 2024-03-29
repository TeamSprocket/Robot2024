
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerSubwooferSpinupTimed extends Command {
  Timer overallTimer = new Timer();
  double duration;
  
  Shooter shooter;
  

  public ScoreSpeakerSubwooferSpinupTimed(Shooter shooter, double duration) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    overallTimer.reset();
    overallTimer.start();


    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterStates.STANDBY);
  }

  @Override
  public boolean isFinished() {
    return overallTimer.get() > duration;
  }
}
