
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.RobotState;
import frc.util.ShuffleboardPIDTuner;

public class runMotor extends SubsystemBase {
 
  private static WPI_TalonFX driveMotorL;
  private static WPI_TalonFX driveMotorR;
  
  public runMotor(int driveMotorLID, int driveMotorRID) {
    ShuffleboardPIDTuner.addSlider("Motor Power", 0.0, 100.0, 0);
    this.driveMotorL = new WPI_TalonFX(driveMotorLID);
    //this.driveMotorR = new WPI_TalonFX(driveMotorRID);
  
  }

  /**
   * puts values of everything like speed and angle into the smartdashboard
   */

  public void periodic() {

  }

  public static void run(double output){
    driveMotorL.set(output);
    //driveMotorR.set(-output);
  }





  




}











