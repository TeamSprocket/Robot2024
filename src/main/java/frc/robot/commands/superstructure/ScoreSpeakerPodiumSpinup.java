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

public class ScoreSpeakerPodiumSpinup extends Command {

  Shooter shooter;
  ShooterPivot shooterPivot;

  public ScoreSpeakerPodiumSpinup(ShooterPivot shooterPivot, Shooter shooter, SwerveDrive swerveDrive) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    shooter.setState(ShooterStates.SPINUP_PODIUM);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
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
