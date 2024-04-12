package frc.robot.commands.superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
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
    Intake intake;
    
    Timer timer = new Timer();

    public ScoreAmp(Elevator elevator, ShooterPivot shooterPivot, Shooter shooter, Intake intake) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.intake = intake;
      
        addRequirements(elevator, shooterPivot, shooter, intake);
    }

    @Override
    public void initialize(){
      elevator.setState(ElevatorStates.AMP); // put back  
      shooterPivot.setState(ShooterPivotStates.AMP);
      shooter.setState(ShooterStates.SCORE_AMP);
      intake.setState(IntakeStates.AMP);
    }


    
    @Override
    public void end(boolean interrupted) {
      timer.reset();
      timer.start();
      
      elevator.setState(ElevatorStates.STOWED); // put back
      shooterPivot.setState(ShooterPivotStates.STOWED);
      shooter.setState(ShooterStates.STANDBY);
      
      if (timer.get() > 0.5) {
        intake.setState(IntakeStates.STOWED);
      }

      timer.stop();
    }

    @Override
    public boolean isFinished() {
      return timer.get() > Constants.Superstructure.kScoreAmpDuration;
    }
    
   

    


}
