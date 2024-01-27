
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

public class RunMotor extends SubsystemBase {
 
  private CANSparkMax shooterLeft = new CANSparkMax(11 , MotorType.kBrushless); //top
  private CANSparkMax shooterRight = new CANSparkMax(12, MotorType.kBrushless); //bottom
  private WPI_TalonFX indexer = new WPI_TalonFX(45);

  public RunMotor() {
    // POWER CYCLE IF NOT WORK also set to 0.462 for green wheel prototype
    shooterRight.setInverted(true); //true for Prototype 1 (flywheel) but false for Prototype 2 (updown)
    shooterLeft.setInverted(true); //false for Prorotype 1
  }
  /**
   * puts values of everything like speed and agle into the smartdashboard
   */

  public void periodic() {
    
  }

  public void run(double output, double indexerPower){
    shooterLeft.set(output);
    shooterRight.set(output);
    indexer.set(-indexerPower);
  }





  




}











