
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerSubwooferSpinup extends Command {
  Shooter shooter;
  ShooterPivot shooterPivot;

  public ScoreSpeakerSubwooferSpinup(Shooter shooter, ShooterPivot shooterPivot) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void initialize() {
    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
