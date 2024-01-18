
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist.WristStates;

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

  Wrist wrist = new Wrist();

  Timer timer = new Timer();

  

  public Superstructure() {
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
        // ELevator: NONE
        wrist.setState(WristStates.NONE);
        // Shooter: NONE
        // Intake: NONE
      break;
      

      case STOWED:
        // ELevator: STOWED
        wrist.setState(WristStates.STOWED);
        // Shooter: STANDBY, NONE if still coasting
        // Intake: STOWED
      break;
      

      case INTAKE:
        // ELevator: STOWED
        wrist.setState(WristStates.STOWED);
        // Shooter: STANDBY
        // Intake: INTAKE
      break;
      

      case HANDOFF:
        // ELevator: HANDOFF
        wrist.setState(WristStates.HANDOFF);
        // Shooter: HANDOFF
        // Intake: HANDOFF
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
