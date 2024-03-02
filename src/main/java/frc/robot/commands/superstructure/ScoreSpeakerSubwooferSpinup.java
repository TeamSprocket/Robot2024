
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerSubwooferSpinup extends Command {
  Shooter shooter;

  public ScoreSpeakerSubwooferSpinup(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
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
    return false;
  }
}
