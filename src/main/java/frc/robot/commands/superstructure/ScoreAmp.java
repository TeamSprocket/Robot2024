package frc.robot.commands.superstructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.robot.subsystems.SwerveDrive;

import frc.robot.subsystems.SwerveDrive.*;

public class ScoreAmp extends Command {

    Superstructure superstructure;

    public ScoreAmp(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize(){
        superstructure.setState(SSStates.WAIT_AMP);
    }

    @Override
    public void execute() {
    }
    
    public boolean isFinished(){
      return superstructure.getState() == (SSStates.STOWED);
    }

    public void end(){
      superstructure.setState(SSStates.STOWED);
    }
        

    


}