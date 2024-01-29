
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.Util;

public class Shooter extends SubsystemBase {
  // NONE - no motor output, STANDBY - resist gamepiece movement in indexer, HANDOFF - intaking from intake, SPINUP - spinup shooter for speaker scoring
  public static enum ShooterStates {
    NONE,
    STANDBY,
    HANDOFF,
    SPINUP, 
    SCORE_SPEAKER,
    SCORE_SPEAKER_HIGH,
    SCORE_AMP
  }
  private ShooterStates state = ShooterStates.NONE;
  private ShooterStates lastState = ShooterStates.NONE;

  // Motors
  WPI_TalonFX shooterMotor = new WPI_TalonFX(RobotMap.Shooter.SHOOTER);
  WPI_TalonFX indexerMotor = new WPI_TalonFX(RobotMap.Shooter.INDEXER);

  PIDController shooterPID = new PIDController(Constants.Shooter.kPShooter, Constants.Shooter.kIShooter, Constants.Shooter.kDShooter);
  PIDController indexerPID = new PIDController(Constants.Shooter.kPIndexer, Constants.Shooter.kIIndexer, Constants.Shooter.kDIndexer);

  DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

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
        if (lastState == ShooterStates.HANDOFF) {
          indexerPID.setSetpoint(indexerMotor.getSelectedSensorPosition());
        }
        holdIndexerPosition();

        shooterMotor.set(0);
        break;



      case HANDOFF:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedHandoff);
        shooterMotor.set(0);
        break;


      
      case SPINUP:
        holdIndexerPosition();
        // shooterPID.setSetpoint(getSpinupVelocityMPS()); //TARGET FROM LIMELIGHT
      break;


      case SCORE_SPEAKER:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);
      break;

      
      case SCORE_SPEAKER_HIGH:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);
      break;


      case SCORE_AMP:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreAmp);
      break;

      
    }

    
    // Update lastState
    lastState = state;
  }





  // Methods
  public void setState(ShooterStates state) {
      this.state = state;
  }





  public ShooterStates getState() {
      return state;
  }

  public double getShooterRPS() {
    double shooterVelocityCounts = shooterMotor.getSelectedSensorVelocity();
    double rpm = Conversions.falconToRPM(shooterVelocityCounts, Constants.Shooter.kShooterGearRatio);
    double rps = rpm / 60.0;
    return rps;
  }

  public double getShooterMPS() {
    double rps = getShooterRPS();
    double mps = rps * Constants.Shooter.kShooterWheelDiameter * Math.PI;
    return mps;
  }

  public double getShooterTargetMPS() {
    
  }






  public boolean beamBroken() {
    return !beamBreak.get();
  }

  public void holdIndexerPosition() {
    double indexerMotorOutput = indexerPID.calculate(indexerMotor.getSelectedSensorPosition());
    indexerMotor.set(indexerMotorOutput);
  }

  public boolean atGoalShooter() {
    double goal = shooterPID.getSetpoint();
    boolean inRange = Util.inRange(getShooterMPS(), (goal - Constants.Shooter.kAtGoalTolerance), (goal + Constants.Shooter.kAtGoalTolerance));
    return inRange;
  }


  


}
