
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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
  TalonFX shooterMotor = new TalonFX(RobotMap.Shooter.SHOOTER);
  TalonFX indexerMotor = new TalonFX(RobotMap.Shooter.INDEXER);

  PIDController shooterPID = new PIDController(Constants.Shooter.kPShooter, Constants.Shooter.kIShooter, Constants.Shooter.kDShooter);
  PIDController indexerPID = new PIDController(Constants.Shooter.kPIndexer, Constants.Shooter.kIIndexer, Constants.Shooter.kDIndexer);

  DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

  Supplier<Translation2d> botPoseSupplier;

  // double shooterInc = 0.0;


  public Shooter(Supplier<Translation2d> botPoseSupplier) {
    shooterMotor.setInverted(Constants.Shooter.kIsShooterInverted);
    indexerMotor.setInverted(Constants.Shooter.kIsIndexerInverted);

    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);

    this.botPoseSupplier = botPoseSupplier;
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
          indexerPID.setSetpoint(indexerMotor.getPosition().getValueAsDouble());
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
        
        // Spin up shooter
        shooterPID.setSetpoint(getShooterTargetMPS());
        shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      break;


      case SCORE_SPEAKER:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

        // Spin up shooter
        shooterPID.setSetpoint(getShooterTargetMPS());
        shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      break;

      
      case SCORE_SPEAKER_HIGH:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

        // Spin up shooter
        shooterPID.setSetpoint(getShooterTargetMPS());
        shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      break;


      case SCORE_AMP:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreAmp);
        shooterMotor.set(Constants.Shooter.kShooterSpeedScoreAmp);
      break;

      
    }

    clearStickyFaults();
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
    double shooterVelocityCounts = shooterMotor.getVelocity().getValueAsDouble();
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
    double dist = Conversions.poseToDistance(botPoseSupplier.get(), Constants.ShootingSetpoints.targetPoint);
    double targetMPS = Constants.ShootingSetpoints.getValues(dist)[1];
    return targetMPS;
  }






  public boolean beamBroken() {
    return !beamBreak.get();
  }

  public void holdIndexerPosition() {
    double indexerMotorOutput = indexerPID.calculate(indexerMotor.getPosition().getValueAsDouble());
    indexerMotor.set(indexerMotorOutput);
  }

  public boolean atGoalShooter() {
    double goal = shooterPID.getSetpoint();
    boolean inRange = Util.inRange(getShooterMPS(), (goal - Constants.Shooter.kAtGoalTolerance), (goal + Constants.Shooter.kAtGoalTolerance));
    return inRange;
  }

  public void clearStickyFaults() {
    shooterMotor.clearStickyFaults();
    indexerMotor.clearStickyFaults();
  }
  


}
