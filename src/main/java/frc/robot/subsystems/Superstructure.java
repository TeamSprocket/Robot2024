
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  Timer timer = new Timer();

  

  public Superstructure() {
    timer.reset();

  }

  // ELevator: NONE, STOWED, HANDOFF, SPEAKER, SPEAKER_HIGH, AMP, MANUAL
  // Wrist: NONE, STOWED, HANDOFF, SPEAKER, SPEAKER_HIGH, AMP, MANUAL
  // Shooter: NONE, STANDBY, HANDOFF, SPINUP, 
  // Intake: NONE



  @Override
  public void periodic() {
    switch (state) {
      case NONE:
        // ELevator: NONE
        // Wrist: NONE
        // Shooter: NONE
        // Intake: NONE
      break;
      

      case STOWED:
        // ELevator: STOWED
        // Wrist: STOWED
        // Shooter: STANDBY, NONE if still coasting
        // Intake: STOWED
      break;
      

      case INTAKE:
        // ELevator: STOWED
        // Wrist: STOWED
        // Shooter: STANDBY
        // Intake: INTAKE
      break;
      

      case HANDOFF:
        // ELevator: HANDOFF
        // Wrist: HANDOFF
        // Shooter: HANDOFF
        // Intake: HANDOFF
      break;
      

      case WAIT_SPEAKER:
        // ELevator: SPEAKER
        // Wrist: SPEAKER
        // Shooter: SPINUP
        // Intake: STOWED
      break;
      

      case WAIT_SPEAKER_HIGH:
        // ELevator: SPEAKER_HIGH
        // Wrist: SPEAKER
        // Shooter: SPINUP
        // Intake: STOWED
      break;
      

      case WAIT_AMP:
        // ELevator: AMP
        // Wrist: AMP
        // Shooter: 
        // Intake: 
      break;
      

      case SCORE_SPEAKER:
        // ELevator: SPEAKER_HIGH
        // Wrist: SPEAKER
        // Shooter: SCORE_SPEAKER
        // Intake: 
      break;
      

      case SCORE_SPEAKER_HIGH:
        // ELevator: 
        // Wrist: 
        // Shooter: 
        // Intake: 
      break;
      

      case SCORE_AMP:
        // ELevator: 
        // Wrist: 
        // Shooter: SCORE_AMP
        // Intake: 
      break;
      

      case MANUAL:
        // ELevator: 
        // Wrist: 
        // Shooter: 
        // Intake: 
      break;
      

    }
  }




  // Methods

  /**
   * Sets the superstructure target state
   * @param state Target state
   */
  public void setState(SSStates state) {
      this.state = state;
  }








}
