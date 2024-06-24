package frc.robot.commands.superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.superstructure.IntakeNote.IntakeCommandStates;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Superstructure.SSStates;

public class WaitAmp extends Command {

    Elevator elevator;
    ShooterPivot shooterPivot;
    Intake intake;
    Timer timer = new Timer();
        
    public WaitAmp(Elevator elevator, ShooterPivot shooterPivot, Intake intake) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.intake = intake;
      
        addRequirements(elevator, shooterPivot, intake);
    }

    @Override
    public void initialize(){
      elevator.setState(ElevatorStates.AMP); // put back  
      shooterPivot.setState(ShooterPivotStates.AMP);
      intake.setState(IntakeStates.AMP);
    }


    @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.start();

    elevator.setState(ElevatorStates.STOWED); // put back
    shooterPivot.setState(ShooterPivotStates.STOWED);
    
    if (timer.get() > 0.5) {
      intake.setState(IntakeStates.STOWED);
    }
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
    
   

    


}
