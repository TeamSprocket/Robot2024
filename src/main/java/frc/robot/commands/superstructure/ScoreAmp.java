package frc.robot.commands.superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SSStates;

public class ScoreAmp extends InstantCommand {

    Superstructure superstructure;

    public ScoreAmp(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize(){
      if (superstructure.getState() != SSStates.WAIT_AMP) { // At amp, stowed
        superstructure.setState(SSStates.WAIT_AMP);
      } else { // Waiting
        superstructure.setState(SSStates.SCORE_AMP);
      }
    }
    
   

    


}
