package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class ShootCrossfieldSpinup extends Command {

  Shooter shooter;
  Intake intake; // check if intake is necessary
  ShooterPivot shooterPivot;
  Timer timer = new Timer();
  Timer scoreTimer = new Timer();
  Boolean isShooting;

  public ShootCrossfieldSpinup(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;
    
    this.isShooting = false;
  }

  @Override
  public void initialize() {
    intake.setState(IntakeStates.SCORE_SPEAKER);
    shooter.setState(ShooterStates.SPINUP_CROSSFIELD);

    timer.reset();
    timer.start();

    scoreTimer.reset();
    scoreTimer.stop();
  }

  @Override
  public void execute() {
    if (timer.get() > 0.2) { // delay for intake move out
      shooterPivot.setState(ShooterPivotStates.CROSSFIELD);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return scoreTimer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec;
  }
}
