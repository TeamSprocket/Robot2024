package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist.WristStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Intake.IntakeStates;

public class Superstructure extends SubsystemBase {
  public static enum SSStates {
    NONE,
    STOWED,
    INTAKE,
    WAIT_HANDOFF, HANDOFF,
    WAIT_SPEAKER, WAIT_SPEAKER_HIGH, WAIT_AMP,
    SCORE_SPEAKER, SCORE_SPEAKER_HIGH, SCORE_AMP,
    CLIMB
  }
  public SSStates state = SSStates.NONE;
  public SSStates lastState = SSStates.NONE;

  Timer timer = new Timer();

  Elevator elevator;
  Wrist wrist;
  Shooter shooter;
  Intake intake;

  public Superstructure(Elevator elevator, Wrist wrist, Shooter shooter, Intake intake) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
    this.intake = intake;

    timer.reset();
  }

  // ELevator: NONE, STOWED, HANDOFF, SPEAKER, SPEAKER_HIGH, AMP, MANUAL
  // Wrist: NONE, STOWED, HANDOFF, SPEAKER, SPEAKER_HIGH, AMP, MANUAL
  // Shooter: NONE, STANDBY, HANDOFF, SPINUP, SCORE_SPEAKER, SCORE_AMP
  // Intake: NONE, STOWED, INTAKE, WAIT_HANDOFF, HANDOFF



  @Override
  public void periodic() {
    switch (state) {
      case NONE:
        elevator.setState(ElevatorStates.NONE);
        wrist.setState(WristStates.NONE);
        shooter.setState(ShooterStates.NONE);
        intake.setState(IntakeStates.NONE);
      break;
      

      case STOWED:
        elevator.setState(ElevatorStates.STOWED);
        wrist.setState(WristStates.STOWED);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case INTAKE:
        // Reset tolerance timer
        if (lastState != SSStates.INTAKE) {
          timer.reset();
          timer.stop();
        }

        // Start tolerance timer
        if (intake.hasDetectedNote()) {
          timer.start();
        }

        elevator.setState(ElevatorStates.STOWED);
        wrist.setState(WristStates.STOWED);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.INTAKE);

        if (timer.get() > Constants.Superstructure.kIntakeTimeToStowToleranceSec) {
          setState(SSStates.STOWED);
        } 
      break;
      

      case WAIT_HANDOFF:
      if (allElementsAtGoalNoShooter()) {
        timer.start(); 
      } else {
        timer.reset();
        timer.stop();
      }

        elevator.setState(ElevatorStates.HANDOFF);
        wrist.setState(WristStates.HANDOFF);
        shooter.setState(ShooterStates.HANDOFF);
        intake.setState(IntakeStates.WAIT_HANDOFF);

        if (timer.get() > Constants.Superstructure.kWaitHandoffTimeToleranceSec) {
          setState(SSStates.HANDOFF);
        }

      break;


      case HANDOFF:
        if (shooter.beamBroken()) {
          // Note in shooter 
          timer.start();
        } else {
          timer.reset();
          timer.stop();
        }

        elevator.setState(ElevatorStates.HANDOFF);
        wrist.setState(WristStates.HANDOFF);
        shooter.setState(ShooterStates.HANDOFF);
        intake.setState(IntakeStates.HANDOFF);

        if (timer.get() >= Constants.Superstructure.kWaitBeambreakTimeToleranceSec) {
          setState(SSStates.STOWED);
        }
        
      break;
      

      case WAIT_SPEAKER:
        elevator.setState(ElevatorStates.SPEAKER);
        wrist.setState(WristStates.SPEAKER);
        shooter.setState(ShooterStates.SPINUP);
        intake.setState(IntakeStates.STOWED);

        // Transitioner
      break;
      

      case WAIT_SPEAKER_HIGH:
        elevator.setState(ElevatorStates.SPEAKER_HIGH);
        wrist.setState(WristStates.SPEAKER_HIGH);
        shooter.setState(ShooterStates.SPINUP);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case WAIT_AMP:
        elevator.setState(ElevatorStates.AMP);
        wrist.setState(WristStates.AMP);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case SCORE_SPEAKER:
        elevator.setState(ElevatorStates.SPEAKER);
        wrist.setState(WristStates.SPEAKER);
        shooter.setState(ShooterStates.SCORE_SPEAKER);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case SCORE_SPEAKER_HIGH:
        elevator.setState(ElevatorStates.SPEAKER_HIGH);
        wrist.setState(WristStates.SPEAKER_HIGH);
        shooter.setState(ShooterStates.SCORE_SPEAKER_HIGH);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case SCORE_AMP:
        elevator.setState(ElevatorStates.AMP);
        wrist.setState(WristStates.AMP);
        shooter.setState(ShooterStates.SCORE_AMP);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case CLIMB:
        elevator.setState(ElevatorStates.CLIMB);
        wrist.setState(WristStates.CLIMB);
        shooter.setState(ShooterStates.NONE);
        intake.setState(IntakeStates.STOWED);
      break;
    }


    lastState = state;
  }




  // Methods

  /**
   * Sets the superstructure target state
   * @param state Target state
   */
  public void setState(SSStates state) {
      this.state = state;
  }

  public SSStates getState() {
    return state;
  }

  /**
   * @return SHOOTER INCLUDED, if all elements are within tolerance of their goals
   */
  public boolean allElementsAtGoal() {
    return elevator.atGoal() && wrist.atGoal() && intake.atGoal() && shooter.atGoalShooter();
  }


  /**
   * @return SHOOTER NOT INCLUDED, if all elements are within tolerance of their goals
   */
  public boolean allElementsAtGoalNoShooter() {
    return elevator.atGoal() && wrist.atGoal() && intake.atGoal();
  }








}
