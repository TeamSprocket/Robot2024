package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;

public class Superstructure extends SubsystemBase {
  public static enum SSStates {
    NONE,
    STOWED,
    INTAKE,
    WAIT_SPEAKER_SUBWOOFER, WAIT_SPEAKER_PODIUM,
    SCORE,
    EJECT_NOTE,
    CROSSFIELD
  }
  public SSStates currentState = SSStates.NONE;
  public SSStates lastState = SSStates.NONE;
  public SSStates wantedState = SSStates.NONE;

  ShooterPivot shooterPivot;
  Shooter shooter;
  Intake intake;
  private Timer spinupTimer = new Timer();
  private Timer intakeTimer = new Timer();

  public Superstructure(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;

    spinupTimer.reset();
    intakeTimer.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Superstructure State [SSS]", currentState.toString());
    handleStateChange();
  }

  // Methods

  public void handleStateChange() {
    lastState = currentState;
    currentState = wantedState;

    switch (currentState) {

      case NONE:
      break;
      
      case STOWED:
      default:
        stowed();
        break;
      
      case INTAKE:
        intake();
      break;

      case WAIT_SPEAKER_SUBWOOFER:
        spinupSub();
      break;
      
      case WAIT_SPEAKER_PODIUM:
        spinupPod();
      break;
      
      case SCORE:
        score();
      break;

      case EJECT_NOTE:
        ejectNote();
      break;

      case CROSSFIELD:
        crossfield();
      break;
    }
  }

  // ------ states ------

  private void stowed() { //
    shooterPivot.setState(ShooterPivotStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
    intake.setState(IntakeStates.STOWED);
  }

  private void intake() { // TODO: RECODE
    intakeTimer.restart(); // TODO: check if this works

    intake.setState(IntakeStates.INTAKE);
    shooter.setState(ShooterStates.INTAKE_ACCEL);
    shooterPivot.setState(ShooterPivotStates.INTAKE);

    if (intakeTimer.hasElapsed(0.5)) {
      System.out.println("INTAKING INTAKING INTAKING");
      shooter.setState(ShooterStates.INTAKE);
      
      if (shooter.getState() == ShooterStates.INTAKE && shooter.beamBroken()) {
        System.out.println("FORWARD FORWARD FORWARD");

        intake.setState(IntakeStates.INDEXING);
        // shooterPivot.setState(ShooterPivotStates.INDEXING);
        shooter.setState(ShooterStates.INTAKE_ROLLFORWARD);

        if (shooter.getState() == ShooterStates.INTAKE_ROLLFORWARD && shooter.hasDetectedNoteShooter() && intakeTimer.hasElapsed(0.3)) {
          shooter.setState(ShooterStates.HOLD_NOTE);

          if (shooter.getState() == ShooterStates.HOLD_NOTE && intakeTimer.hasElapsed(0.15)) {
            shooter.setState(ShooterStates.INTAKE_ROLLBACK);

            if (shooter.getState() == ShooterStates.INTAKE_ROLLBACK && shooter.hasNoteRollbackIndexer() && intakeTimer.hasElapsed(0.1)) {
              return;
            }
          }
        }
      }
    }
  }

  private void spinupSub() { //
    spinupTimer.restart();

    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
    intake.setState(IntakeStates.SCORE_SPEAKER_SUBWOOFER);

    if (shooter.atGoalShooter() && spinupTimer.hasElapsed(0.5)) {
      shooter.setIndexerSpeedScoreSpeaker();

      if (spinupTimer.hasElapsed(0.5)) {
        return;
      }
    }
  }

  private void spinupPod() { //
    spinupTimer.restart();
    
    shooter.setState(ShooterStates.SPINUP_PODIUM);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
    intake.setState(IntakeStates.STOWED);

    if (shooter.atGoalShooter() && spinupTimer.hasElapsed(0.3) && shooterPivot.atGoal()) { // TODO: read values to check speed
      shooter.setIndexerSpeedScoreSpeaker();

      if (spinupTimer.hasElapsed(0.5)) {
        return;
      }
    }
  }

  private void score() { // make sure pivot stays in correct position

    if (shooter.atGoalShooter() && spinupTimer.hasElapsed(0.3)) {
      shooter.setIndexerSpeedScoreSpeaker();
    }
    // shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
    // shooter.setState(ShooterStates.SCORE);
    intake.setState(IntakeStates.STOWED);
  }

  private void ejectNote() { //
    intake.setState(IntakeStates.EJECT_NOTE);
    if (spinupTimer.hasElapsed(0.5)) {
      shooterPivot.setState(ShooterPivotStates.EJECT_NOTE);

      if (spinupTimer.hasElapsed(1)){
        shooter.setState(ShooterStates.EJECT_NOTE);
      }
    }
  }

  private void crossfield() {
    spinupTimer.restart();

    intake.setState(IntakeStates.CROSSFIELD);
    shooter.setState(ShooterStates.SPINUP_CROSSFIELD);

    if (spinupTimer.hasElapsed(0.2)) { // delay for intake move out
      shooterPivot.setState(ShooterPivotStates.CROSSFIELD);

      if (shooter.atGoalShooter() && spinupTimer.hasElapsed(0.3) && shooterPivot.atGoal()) {
        shooter.setIndexerSpeedScoreSpeaker();
      }
    }
  }

  // ------ commands -------

  /**
   * Sets the superstructure target state
   * @param currentState Target state
   */
  public Command setState(SSStates wantedState) {
      return new InstantCommand(() -> this.wantedState = wantedState);
  }

  // ------ methods ------

  public boolean waitTimed(double duration) {
    Timer timer = new Timer();
    System.out.println("WAITTIMERWAITTIMER/nWAITTIMERWAITTIMER/nWAITTIMERWAITTIMER/nWAITTIMERWAITTIMER/nWAITTIMERWAITTIMER/n");
    return timer.get() > duration;
  }

  public SSStates getCurrentState() {
      return currentState;
  }
}
