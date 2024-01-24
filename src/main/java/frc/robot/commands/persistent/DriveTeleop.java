
package frc.robot.commands.persistent;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RunMotor;
import frc.util.ShuffleboardPIDTuner;

public class DriveTeleop extends CommandBase {

  RunMotor runMotor;
  
  public DriveTeleop(RunMotor runMotor) {
    this.runMotor = runMotor;
    
    ShuffleboardPIDTuner.addSlider("Motor Power", -1.0, 1.0, 0);

    addRequirements(runMotor);
  }

  @Override
  public void initialize() {}
  
  
  @Override
  public void execute() {
    runMotor.run(ShuffleboardPIDTuner.get("Motor Power"));
  }
  

}
