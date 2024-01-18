
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  // NONE - no motor output, STANDBY - resist gamepiece movement in indexer, HANDOFF - intaking from intake, SPINUP - spinup shooter for speaker scoring
  public static enum ShooterStates {
    NONE,
    STANDBY,
    HANDOFF,
    SPINUP
  }
  private ShooterStates state = ShooterStates.NONE;
  private ShooterStates lastState = ShooterStates.NONE;

  // Motors
  WPI_TalonFX shooterMotor = new WPI_TalonFX(RobotMap.Shooter.SHOOTER);
  WPI_TalonFX indexerMotor = new WPI_TalonFX(RobotMap.Shooter.INDEXER);

  PIDController shooterPID = new PIDController(Constants.Shooter.kPShooter, Constants.Shooter.kIShooter, Constants.Shooter.kDShooter);
  PIDController indexerPID = new PIDController(Constants.Shooter.kPIndexer, Constants.Shooter.kIIndexer, Constants.Shooter.kDIndexer);

  // double shooterInc = 0.0;


  public Shooter() {
    shooterMotor.setInverted(Constants.Shooter.kIsShooterInverted);
    indexerMotor.setInverted(Constants.Shooter.kIsIndexerInverted);

    shooterMotor.setNeutralMode(NeutralMode.Coast);
    indexerMotor.setNeutralMode(NeutralMode.Brake);

  }


  @Override
  public void periodic() {
    switch (state) {
      case NONE:
        shooterMotor.set(0);
        indexerMotor.set(0);
        break;



      case STANDBY:
        if (lastState != ShooterStates.STANDBY) {
          indexerPID.setSetpoint(indexerMotor.getSelectedSensorPosition());
        }
        double indexerMotorOutput = indexerPID.calculate(indexerMotor.getSelectedSensorPosition());
        indexerMotor.set(indexerMotorOutput);

        break;



      case HANDOFF:
        
        break;


      
      case SPINUP:
        // Set index to constant forward thing idk kms nightmarenightmarenightmare
        // shooterPID.setSetpoint(getSpinupVelocityMPS()); //TARGET FROM LIMELIGHT
      break;
    
      
    }

    
    lastState = state;
  }





  // Methods
  public void setState(ShooterStates state) {
      this.state = state;
  }

  public boolean atVelocity() {
    return false;
  }


  // Will require testing to find priority angle vs velocity, for now using arbitrary solution

  /**
   * 
   * @param targetX Relative x distance to target in meters
   * @param targetY Relative y distance to target in meters 
   * @param angle Initial launch angle IN DEGREES
   * @return Spinup required initial launch velocity, -1 IF UNDEF
   */ // NOTE: If units changed from meters, gravity constant must be updated 
  public double getSpinupVelocityMPS(double targetX, double targetY, double angle) {
    try {
      double rad = Math.toRadians(angle);
      // a, b, c vars to reduce length of lines
      double a = 9.8067 * Math.pow(targetX, 2);
      double b = ((targetX * Math.tan(rad)) - targetY);
      double c = 2.0 * Math.pow(Math.cos(rad), 2);
      double result = Math.sqrt(a / (b * c));
      return result;

    } catch (Exception e) {
      return -1.0;
    }
  }


}
