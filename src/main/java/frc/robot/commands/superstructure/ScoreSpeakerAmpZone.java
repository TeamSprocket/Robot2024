
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Conversions;

public class ScoreSpeakerAmpZone extends Command {

  Superstructure superstructure;
  SwerveDrive swerveDrive;
  Timer timer = new Timer();

  public ScoreSpeakerAmpZone(Superstructure superstructure, SwerveDrive swerveDrive) {
    this.superstructure = superstructure;
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    superstructure.setState(SSStates.WAIT_SPEAKER_AMP_ZONE);
  }

  @Override
  public void execute() {
    swerveDrive.setTargetHeadingRad(Conversions.poseToTargetHeadingRad(swerveDrive.getPose().getTranslation(), Constants.ShootingSetpoints.targetPoint));
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setState(SSStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return superstructure.getState() == SSStates.STOWED;
  }
}
