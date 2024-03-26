package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Shintake.ShintakeStates;

/*
 * initialize Shintake
 */

public class ShootAmp extends Command {
    Shintake shintake = new Shintake();
    public ShootAmp(Shintake shintake){
        this.shintake = shintake;
    }

// Called when the command is initially scheduled.
  /**
   * when started, the command will call SCORE_AMP
   * SCORE_AMP is 
   */
  @Override
  public void initialize() {
    shintake.setState(ShintakeStates.SCORE_AMP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  /**
   * once something happens, the instake will be at the stowed state
   */
  @Override
  public void end(boolean interrupted) {
    shintake.setState(ShintakeStates.STOWED);
  }

  // Returns true when the command should end.
  /**
   * finished
   */
  @Override
  public boolean isFinished() {
    return (!shintake.hasNote());
  }
}


