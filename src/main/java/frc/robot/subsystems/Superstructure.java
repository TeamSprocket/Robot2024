package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  private void stowed() {
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

  private void spinupSub() {
    shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
    intake.setState(IntakeStates.SCORE_SPEAKER_SUBWOOFER);
  }

  private void spinupPod() {
    shooter.setState(ShooterStates.SPINUP_PODIUM);
    intake.setState(IntakeStates.SCORE_SPEAKER_PODIUM);
    shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
  }

  private void ejectNote() {
    intake.setState(IntakeStates.EJECT_NOTE);
    shooterPivot.setState(ShooterPivotStates.EJECT_NOTE);
    shooter.setState(ShooterStates.EJECT_NOTE);
  }

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

  public Command spinupSubCmd() {
    return setState(SSStates.WAIT_SPEAKER_SUBWOOFER);
  }

  public Command spinupPodCmd() {
    return setState(SSStates.WAIT_SPEAKER_PODIUM);
  }
  
  public Command ejectNoteCmd() {
    return setState(SSStates.EJECT_NOTE);
  }

  public Command intakeCmd() {
    return new SequentialCommandGroup(
      setState(SSStates.INTAKE),
      new WaitUntilCommand(() -> shooter.beamBroken()),
      setState(SSStates.STOWED),
      new WaitCommand(0.5),
      setState(SSStates.INTAKE_BACK),
      setState(SSStates.STOWED)
    );
  }
  // ------ methods ------

  public SSStates getCurrentState() {
      return currentState;
  }
}