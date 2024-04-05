package frc.robot.commands.superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SSStates;

public class ScoreAmp extends InstantCommand {

    Elevator elevator;
    ShooterPivot shooterPivot;
    Shooter shooter;
    
    Timer timer = new Timer();

    public ScoreAmp(Elevator elevator, ShooterPivot shooterPivot, Shooter shooter) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
      
        addRequirements(elevator, shooterPivot, shooter);
    }

    @Override
    public void initialize(){
      elevator.setState(ElevatorStates.AMP); // put back  
      shooterPivot.setState(ShooterPivotStates.AMP);
      shooter.setState(ShooterStates.SCORE_AMP);

      timer.reset();
      timer.start();
    }


    
    @Override
    public void end(boolean interrupted) {
      timer.stop();
      elevator.setState(ElevatorStates.STOWED); // put back
      shooterPivot.setState(ShooterPivotStates.STOWED);
      shooter.setState(ShooterStates.STANDBY);
    }

    @Override
    public boolean isFinished() {
      return timer.get() > Constants.Superstructure.kScoreAmpDuration;
    }
    
   

    


}
