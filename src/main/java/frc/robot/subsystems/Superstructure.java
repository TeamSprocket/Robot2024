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
    HANDOFF,
    WAIT_SPEAKER, WAIT_SPEAKER_HIGH, WAIT_AMP,
    SCORE_SPEAKER, SCORE_SPEAKER_HIGH, SCORE_AMP,
    MANUAL
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
      

      case HANDOFF:
        if (!shooter.beamBroken()) {
          timer.reset();
          timer.stop();
        } else {
          // Note in shooter 
          timer.start();
        }

        elevator.setState(ElevatorStates.STOWED);
        wrist.setState(WristStates.STOWED);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.INTAKE);
        
      break;
      

      case WAIT_SPEAKER:
        // ELevator: SPEAKER
        wrist.setState(WristStates.SPEAKER);
        // Shooter: SPINUP
        // Intake: STOWED
      break;
      

      case WAIT_SPEAKER_HIGH:
        // ELevator: SPEAKER_HIGH
        wrist.setState(WristStates.SPEAKER);
        // Shooter: SPINUP
        // Intake: STOWED
      break;
      

      case WAIT_AMP:
        // ELevator: AMP
        wrist.setState(WristStates.AMP);
        // Shooter: 
        // Intake: 
      break;
      

      case SCORE_SPEAKER:
        // ELevator: SPEAKER_HIGH
        wrist.setState(WristStates.SPEAKER);
        // Shooter: SCORE_SPEAKER
        // Intake: 
      break;
      

      case SCORE_SPEAKER_HIGH:
        // ELevator: 
        wrist.setState(WristStates.SPEAKER_HIGH); 
        // Shooter: 
        // Intake: 
      break;
      

      case SCORE_AMP:
        // ELevator: 
        wrist.setState(WristStates.AMP);
        // Shooter: SCORE_AMP
        // Intake: 
      break;
      

      case MANUAL:
        // ELevator:  
        wrist.setState(WristStates.MANUAL);
        // Shooter: 
        // Intake: 
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

  public boolean allElementsAtTarget() {
    re
  }








}
