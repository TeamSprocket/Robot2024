
package frc.robot.commands.persistent;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.runMotor;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTeleop extends CommandBase {


  public DriveTeleop() {
    
    ShuffleboardPIDTuner.addSlider("Motor Power", 0.0, 100.0, 0);
  }

  @Override
  public void initialize() {}
  
  
  @Override
  public void execute() {
    runMotor.run(ShuffleboardPIDTuner.get("Motor Power"));
  }
  

}
