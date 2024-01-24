
package frc.robot.commands.persistent;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RunMotor;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTeleop extends CommandBase {

  RunMotor runMotor;
  
  public DriveTeleop(RunMotor runMotor) {
    this.runMotor = runMotor;
    
    ShuffleboardPIDTuner.addSlider("Motor Power", -1.0, 1.0, 0);
    ShuffleboardPIDTuner.addSlider("Indexer Power", -1.0, 1.0, 0);

    addRequirements(runMotor);
  }

  @Override
  public void initialize() {}
  
  
  @Override
  public void execute() {
    runMotor.run(ShuffleboardPIDTuner.get("Motor Power"), ShuffleboardPIDTuner.get("Indexer Power"));
  }
  

}
