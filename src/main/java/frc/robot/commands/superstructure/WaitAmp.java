package frc.robot.commands.superstructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.superstructure.IntakeNoteManual.IntakeCommandStates;
import frc.robot.subsystems.Elevator;
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

    public WaitAmp(Elevator elevator, ShooterPivot shooterPivot) {
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
      
        addRequirements(elevator, shooterPivot);
    }

    @Override
    public void initialize(){
      elevator.setState(ElevatorStates.AMP); // put back  
      shooterPivot.setState(ShooterPivotStates.AMP);
    }


    @Override
  public void end(boolean interrupted) {
    elevator.setState(ElevatorStates.STOWED); // put back
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
    
   

    


}
