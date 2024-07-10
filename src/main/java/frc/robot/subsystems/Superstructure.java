package frc.robot.subsystems;

import org.ejml.dense.row.decompose.hessenberg.HessenbergSimilarDecomposition_CDRM;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.superstructure.IntakeNote.IntakeCommandStates;
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
  private Timer timer = new Timer();

  public Superstructure(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;
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

  private void intake() { // TEST NEW TIMER CODE

    System.out.println("STARTING STARTING STARTING");
    Timer timer = new Timer();
    Timer accelTimer = new Timer();
    IntakeCommandStates state = IntakeCommandStates.ACCEL;
    
    // intake.setState(IntakeStates.INTAKE);
    // shooter.setState(ShooterStates.INTAKE_ACCEL);
    // shooterPivot.setState(ShooterPivotStates.INTAKE);

    // if (shooter.getState() == ShooterStates.INTAKE_ACCEL && waitTimed(0.5)) {
    //   shooter.setState(ShooterStates.INTAKE);

    //   if (shooter.getState() == ShooterStates.INTAKE && shooter.beamBroken()) {
    //     intake.setState(IntakeStates.INDEXING);
    //     shooterPivot.setState(ShooterPivotStates.INDEXING);
    //     shooter.setState(ShooterStates.INTAKE_ROLLFORWARD);

    //     if (shooter.getState() == ShooterStates.INTAKE_ROLLFORWARD && waitTimed(0.3)) {
    //       shooter.setState(ShooterStates.HOLD_NOTE);
          
    //       if (shooter.getState() == ShooterStates.HOLD_NOTE && waitTimed(0.15)) {
    //         shooter.setState(ShooterStates.INTAKE_ROLLBACK);

    //         if (shooter.getState() == ShooterStates.INTAKE_ROLLBACK && waitTimed(0.1) && shooter.hasNoteRollbackIndexer()) {
    //           return;
    //         }
    //       }
    //     }
    //   }
    // }

    state = IntakeCommandStates.ACCEL;

    intake.setState(IntakeStates.INTAKE);
    shooter.setState(ShooterStates.INTAKE_ACCEL);
    shooterPivot.setState(ShooterPivotStates.INTAKE);

    timer.reset();
    timer.stop();

    accelTimer.reset();
    accelTimer.start();

    if (state == IntakeCommandStates.ACCEL && accelTimer.get() > 0.5) {
      System.out.println("INTAKING INTAKING INTAKING");
      state = IntakeCommandStates.INTAKE;
      shooter.setState(ShooterStates.INTAKE);
    }

    if (state == IntakeCommandStates.INTAKE && shooter.beamBroken()) {
      System.out.println("FORWARD FORWARD FORWARD");
      state = IntakeCommandStates.ROLLFORWARD;
      intake.setState(IntakeStates.INDEXING);
      // shooterPivot.setState(ShooterPivotStates.INDEXING);
      shooter.setState(ShooterStates.INTAKE_ROLLFORWARD);
    }

    if (state == IntakeCommandStates.ROLLFORWARD && shooter.hasDetectedNoteShooter()) {
      timer.start();
    }

    if (state == IntakeCommandStates.ROLLFORWARD && timer.get() > Constants.Superstructure.kIndexerIntakeRollForwardTimeSec) {
      state = IntakeCommandStates.WAIT;
      timer.stop();
      timer.reset();
      timer.start();
      shooter.setState(ShooterStates.HOLD_NOTE);
    }

    if (state == IntakeCommandStates.WAIT && timer.get() > Constants.Superstructure.kRollForwardtoRollBackWaitTime) {
      timer.stop();
      timer.reset();
      state = IntakeCommandStates.ROLLBACK;
      shooter.setState(ShooterStates.INTAKE_ROLLBACK);
    }

    if (state == IntakeCommandStates.ROLLBACK && shooter.hasNoteRollbackIndexer()) {
      timer.start();
    }
    
    if (state == IntakeCommandStates.ROLLBACK && timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec) {
      state = IntakeCommandStates.DONE;
    }
  }

  private void spinupSub() { //
    timer.start();

    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
    intake.setState(IntakeStates.SCORE_SPEAKER_SUBWOOFER);

    if (shooter.atGoalShooter() && timer.hasElapsed(0.4)) {
      System.out.println("RUN RUN RUN RUN RUN RUN");
      shooter.setIndexerSpeedScoreSpeaker();

      if (timer.hasElapsed(0.5)) {
        return;
      }
    }
  }

  private void spinupPod() { //
    shooter.setState(ShooterStates.SPINUP_PODIUM);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
    intake.setState(IntakeStates.STOWED);

    if (shooter.atGoalShooter() && waitTimed(0.3)) {
      shooter.setIndexerSpeedScoreSpeaker();

      if (waitTimed(0.5)) {
        return;
      }
    }
  }

  private void score() { // make sure pivot stays in correct position

    if (shooter.atGoalShooter() && waitTimed(0.3)) {
      shooter.setIndexerSpeedScoreSpeaker();
    }
    // shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
    // shooter.setState(ShooterStates.SCORE);
    intake.setState(IntakeStates.STOWED);
  }

  private void ejectNote() { //
    intake.setState(IntakeStates.EJECT_NOTE);
    if (waitTimed(0.5)) {
      shooterPivot.setState(ShooterPivotStates.EJECT_NOTE);
    }
    if (waitTimed(1)){
      shooter.setState(ShooterStates.EJECT_NOTE);
    }
  }

  private void crossfield() {
    intake.setState(IntakeStates.SCORE_SPEAKER);
    shooter.setState(ShooterStates.SPINUP_CROSSFIELD);

    if (waitTimed(0.2)) { // delay for intake move out
      shooterPivot.setState(ShooterPivotStates.CROSSFIELD);
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
