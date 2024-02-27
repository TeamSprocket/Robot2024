package frc.robot.commands.macro;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SSStates;


public class ManualClimb extends Command{
    Superstructure superstructure;

    public ManualClimb(Superstructure superstructure){
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(SSStates.CLIMB);
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
