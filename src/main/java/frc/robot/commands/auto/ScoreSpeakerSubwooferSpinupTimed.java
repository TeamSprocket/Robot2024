
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerSubwooferSpinupTimed extends Command {
  Shooter shooter;
  ShooterPivot shooterPivot;
  double duration;
  Timer timer = new Timer();

  public ScoreSpeakerSubwooferSpinupTimed(Shooter shooter, ShooterPivot shooterPivot, double duration) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.duration = duration;
  }

  @Override
  public void initialize() {
    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.get() > duration;
  }
}
