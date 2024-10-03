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
    INTAKE_BACK,
    WAIT_SPEAKER_SUBWOOFER, WAIT_SPEAKER_PODIUM,
    EJECT_NOTE,
    CROSSFIELD
  }

  public static enum StowState {
    START,
    WAIT_FOR_INTAKE,
    DONE
  }

  public static enum EjectNoteStates {
    START,
    EXTEND_INTAKE,
    WAIT_FOR_INTAKE,
    PIVOT_SHOOTER,
    DONE
  }

  public SSStates currentState = SSStates.NONE;
  public SSStates lastState = SSStates.NONE;
  public SSStates wantedState = SSStates.NONE;

  public StowState stowState = StowState.START;
  public EjectNoteStates ejectNoteState = EjectNoteStates.START;

  ShooterPivot shooterPivot;
  Shooter shooter;
  Intake intake;

  private Timer timer = new Timer();
  private Timer stowTimer = new Timer();
  private Timer ejectTimer = new Timer();

  public Superstructure(ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;

    timer.restart();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putString("Superstructure State [SSS]", currentState.toString());
    SmartDashboard.putString("Last state [SSS]", lastState.toString());
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

      case INTAKE_BACK:
        intakeBack();
      break;

      case WAIT_SPEAKER_SUBWOOFER:
        spinupSub();
      break;
      
      case WAIT_SPEAKER_PODIUM:
        spinupPod();
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
  // private void stowed() { //

  //   // if (lastState == SSStates.EJECT_NOTE) {
  //     shooter.setState(ShooterStates.STANDBY);
  //     shooterPivot.setState(ShooterPivotStates.STOWED);
  //     while (!timer.hasElapsed(0.5)){}
  //     intake.setState(IntakeStates.STOWED);
  //   // } else {
  //   //   shooterPivot.setState(ShooterPivotStates.STOWED);
  //   //   shooter.setState(ShooterStates.STANDBY);
  //   //   intake.setState(IntakeStates.STOWED);
  //   // }
  // }

  private void stowed() {
    // switch (stowState) {
    //   case START:
    //     shooterPivot.setState(ShooterPivotStates.STOWED);
    //     stowTimer.restart();
    //     stowState = StowState.WAIT_FOR_INTAKE;
    //     break;

    //   case WAIT_FOR_INTAKE:
    //     if (stowTimer.hasElapsed(0.5)) {
    //       intake.setState(IntakeStates.STOWED);
    //       shooter.setState(ShooterStates.STANDBY);
    //       stowState = StowState.DONE;
    //     }
    //     break;

    //   case DONE:
    //     stowTimer.stop();
    //     stowState = StowState.START;
    //     break;
    // }
    
    intake.setState(IntakeStates.STOWED);
    shooterPivot.setState(ShooterPivotStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
  }

  private void intake() {
    intake.setState(IntakeStates.INTAKE);
    shooter.setState(ShooterStates.INTAKE);
    shooterPivot.setState(ShooterPivotStates.INTAKE);
  }
  private void intakeBack() {
    shooter.setIndexerRollBack();
  }

  private void spinupSub() { //
    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
    intake.setState(IntakeStates.SCORE_SPEAKER_SUBWOOFER);
  }

  private void spinupPod() { //
    shooter.setState(ShooterStates.SPINUP_PODIUM);
    intake.setState(IntakeStates.SCORE_SPEAKER_PODIUM);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
  }

  private void ejectNote() { //
    intake.setState(IntakeStates.EJECT_NOTE);
    shooterPivot.setState(ShooterPivotStates.EJECT_NOTE);
    shooter.setState(ShooterStates.EJECT_NOTE);
  }

  // private void ejectNote() {
  //   switch (ejectNoteState) {
  //     case START:
  //         // Start by extending the intake
  //         intake.setState(IntakeStates.EJECT_NOTE);
  //         ejectTimer.restart();
  //         ejectNoteState = EjectNoteStates.EXTEND_INTAKE;
  //         break;

  //     case EXTEND_INTAKE:
  //         // Wait until 0.6 seconds have passed for the intake to extend
  //         if (ejectTimer.hasElapsed(0.6)) {
  //             ejectNoteState = EjectNoteStates.PIVOT_SHOOTER;
  //         }
  //         break;

  //     case PIVOT_SHOOTER:
  //         // Move the shooter pivot and shooter to the eject note position
  //         shooterPivot.setState(ShooterPivotStates.EJECT_NOTE);
  //         shooter.setState(ShooterStates.EJECT_NOTE);
  //         ejectNoteState = EjectNoteStates.DONE;
  //         break;

  //     case DONE:
  //         stowTimer.stop();
  //         ejectNoteState = EjectNoteStates.START;
  //         break;
  //     }
  // }

  private void crossfield() {
    intake.setState(IntakeStates.CROSSFIELD);
    shooter.setState(ShooterStates.SPINUP_CROSSFIELD);
    shooterPivot.setState(ShooterPivotStates.CROSSFIELD);
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

  public SSStates getCurrentState() {
      return currentState;
  }
}