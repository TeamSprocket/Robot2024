
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;

public class Shooter extends SubsystemBase {
  // NONE - no motor output, STANDBY - resist gamepiece movement in indexer, HANDOFF - intaking from intake, SPINUP - spinup shooter for speaker scoring
  public static enum ShooterStates {
    NONE,
    STANDBY,
    INTAKE,
    INTAKE_ACCEL, 
    INTAKE_ROLLBACK,
    // SPINUP, 
    // SCORE_SPEAKER,
    // SCORE_SPEAKER_HIGH,
    SPINUP_SUBWOOFER,
    SPINUP_PODIUM,
    SPINUP_AMP_ZONE,
    SCORE_SPEAKER_SUBWOOFER,
    SCORE_SPEAKER_PODIUM,
    SCORE_SPEAKER_AMP_ZONE,
    SCORE_AMP,

    // JUST IN CASE
    SOURCE
  }
  private ShooterStates state = ShooterStates.NONE;
  private ShooterStates lastState = ShooterStates.NONE;

  // Motors
  TalonFX shooterMotor = new TalonFX(RobotMap.Shooter.SHOOTER_TOP);
  TalonFX shooterFollowerMotor = new TalonFX(RobotMap.Shooter.SHOOTER_BOTTOM);
  CANSparkMax indexerMotor = new CANSparkMax(RobotMap.Shooter.INDEXER, MotorType.kBrushless);

  PIDController shooterPID = new PIDController(Constants.Shooter.kPShooter, Constants.Shooter.kIShooter, Constants.Shooter.kDShooter);
  // PIDController indexerPID = new PIDController(Constants.Shooter.kPIndexer, Constants.Shooter.kIIndexer, Constants.Shooter.kDIndexer);

  // DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

  Supplier<Translation2d> botPoseSupplier;

  SendableChooser<ShooterStates> stateChooser = new SendableChooser<ShooterStates>();

  // double shooterSpeed = 0.0;
  double shooterInc = 0.0;
  double indexerMult = 1.0;


  public Shooter(Supplier<Translation2d> botPoseSupplier) {
    shooterMotor.setInverted(Constants.Shooter.kIsShooterTopInverted);
    shooterFollowerMotor.setInverted(Constants.Shooter.kIsShooterBottomInverted);
    indexerMotor.setInverted(Constants.Shooter.kIsIndexerInverted);

    shooterFollowerMotor.setControl(new StrictFollower(shooterMotor.getDeviceID()));
    
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    indexerMotor.setIdleMode(IdleMode.kBrake);

    this.botPoseSupplier = botPoseSupplier;

    stateChooser.setDefaultOption("NONE", ShooterStates.NONE);
    stateChooser.addOption("STANDBY", ShooterStates.STANDBY);
    stateChooser.addOption("INTAKE", ShooterStates.INTAKE);
    stateChooser.addOption("SPINUP_SUBWOOFER", ShooterStates.SPINUP_SUBWOOFER);
    stateChooser.addOption("SPINUP_PODIUM", ShooterStates.SPINUP_PODIUM);
    stateChooser.addOption("SPINUP_AMP_ZONE", ShooterStates.SPINUP_AMP_ZONE);
    stateChooser.addOption("SCORE_SPEAKER_SUBWOOFER", ShooterStates.SCORE_SPEAKER_SUBWOOFER);
    stateChooser.addOption("SCORE_SPEAKER_PODIUM", ShooterStates.SCORE_SPEAKER_PODIUM);
    stateChooser.addOption("SCORE_SPEAKER_AMP_ZONE", ShooterStates.SCORE_SPEAKER_AMP_ZONE);
    stateChooser.addOption("SCORE_AMP", ShooterStates.SCORE_AMP);
    SmartDashboard.putData("Shooter State Chooser [ST]", stateChooser);
    
    // indexerMotor.set(0.1);

    // ShuffleboardPIDTuner.addSlider("Shooter kP [ST]", 0, 1, 0);
    // ShuffleboardPIDTuner.addSlider("Shooter kD [ST]", 0, 1, 0);
  }


  @Override
  public void periodic() {
    // setState(stateChooser.getSelected());
    postSmartDashboardDebug();

    // shooterPID.setP(ShuffleboardPIDTuner.get("Shooter kP [ST]"));
    // shooterPID.setD(ShuffleboardPIDTuner.get("Shooter kD [ST]"));


    switch (state) {

      case NONE:
        shooterMotor.set(0);
        indexerMotor.set(0);
      break;


      case STANDBY:
        // if (lastState == ShooterStates.HANDOFF) {
        //   indexerPID.setSetpoint(indexerMotor.getRotorPosition().getValueAsDouble());
        // }
        // holdIndexerPosition();
        if (lastState != ShooterStates.STANDBY) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(0);

        shooterInc = 0.0;
        shooterMotor.set(0);
        break;

      case INTAKE:
        if (lastState != ShooterStates.INTAKE) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(Constants.Shooter.kIndexerSpeedIntake); 
        // System.out.println("" + Constants.Shooter.kIndexerSpeedIntake);

        shooterInc = 0.0;
        shooterMotor.set(0);
      break;

      case INTAKE_ACCEL:
        if (lastState != ShooterStates.INTAKE) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(Constants.Shooter.kIndexerSpeedIntake * 0.75); 
        // System.out.println("" + Constants.Shooter.kIndexerSpeedIntake);

        shooterInc = 0.0;
        shooterMotor.set(0);
      break;

      case INTAKE_ROLLBACK:
        if (lastState != ShooterStates.INTAKE_ROLLBACK) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(-0.3); 

        shooterInc = 0.0;
        shooterMotor.set(0);
      break;

        

      
      // case SPINUP:
      //   // holdIndexerPosition();
      //   if (lastState != ShooterStates.SPINUP) {
      //     indexerMotor.setIdleMode(IdleMode.kBrake);
      //   }
        
      //   // Spin up shooter
      //   shooterPID.setSetpoint(getShooterTargetMPS());
      //   shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      // break;


      case SPINUP_SUBWOOFER: 
        if (lastState != ShooterStates.SPINUP_SUBWOOFER) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(0);

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;



      case SPINUP_PODIUM: 
        if (lastState != ShooterStates.SPINUP_PODIUM) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(0);

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerPodium) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;



      case SPINUP_AMP_ZONE: 
        if (lastState != ShooterStates.SPINUP_AMP_ZONE) {
          indexerMotor.setIdleMode(IdleMode.kBrake);
        }
        indexerMotor.set(0);

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerAmpZone) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;







      // case SCORE_SPEAKER:
      //   indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

      //   // Spin up shooter
      //   shooterPID.setSetpoint(getShooterTargetMPS());
      //   shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      // break;

      
      // case SCORE_SPEAKER_HIGH:
      //   indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

      //   // Spin up shooter
      //   shooterPID.setSetpoint(getShooterTargetMPS());
      //   shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      // break;


      case SCORE_SPEAKER_SUBWOOFER:
        if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
          indexerMotor.setIdleMode(IdleMode.kCoast);
        }

        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;



      case SCORE_SPEAKER_PODIUM:
        if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
          indexerMotor.setIdleMode(IdleMode.kCoast);
        }

        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerPodium) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;



      case SCORE_SPEAKER_AMP_ZONE:
        if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
          indexerMotor.setIdleMode(IdleMode.kCoast);
        }

        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerAmpZone) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;

      case SCORE_AMP:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreAmp);
        shooterMotor.set(Constants.Shooter.kShooterSpeedScoreAmp);
      break;

      // JUST IN CASE
      case SOURCE:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedSource);
        shooterMotor.set(0.0);
      break;
      
    }

    // clearStickyFaults();
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

  public double getShooterMPS() {
    double rps = shooterMotor.getRotorVelocity().getValueAsDouble();
    double mps = rps * (Constants.Shooter.kShooterWheelDiameter * Math.PI);
    return mps;
  }

  // public double getShooterTargetMPS() {
  //   double dist = Conversions.poseToDistance(botPoseSupplier.get(), Constants.ShootingSetpoints.targetPoint);
  //   double targetMPS = Constants.ShootingSetpoints.getValues(dist)[1];
  //   return targetMPS;
  // }


  // public double TEMP_HUENEME_ACCELER(double indexerMult) {
    
  // }




  public boolean beamBroken() {
    // return !beamBreak.get();
    return false;
  }

  // public void holdIndexerPosition() {
  //   double indexerMotorOutput = indexerPID.calculate(indexerMotor.getRotorPosition().getValueAsDouble());
  //   indexerMotor.set(indexerMotorOutput);
  // }

  public boolean atGoalShooter() {
    double goal = shooterPID.getSetpoint();
    boolean inRange = Util.inRange(getShooterMPS(), (goal - Constants.Shooter.kAtGoalTolerance), (goal + Constants.Shooter.kAtGoalTolerance));
    return inRange;
  }

  public void clearStickyFaults() {
    shooterMotor.clearStickyFaults();
    shooterFollowerMotor.clearStickyFaults();
    // indexerMotor.clearStickyFaults();
  }

  public boolean hasDetectedNote() {
    return indexerMotor.getOutputCurrent() > Constants.Shooter.kHasNoteCurrentThreshold;
 }


  public void postSmartDashboardDebug() {
    SmartDashboard.putString("State [ST]", state.toString());
    SmartDashboard.putNumber("Shooter MPS [ST]", getShooterMPS());
    SmartDashboard.putNumber("Shooter Target MPS [ST]", shooterPID.getSetpoint());
    SmartDashboard.putBoolean("atGoalShooter [ST]", atGoalShooter());
    SmartDashboard.putNumber("Shooter PID Output [ST]", shooterInc);
    SmartDashboard.putBoolean("Beam Broken [ST]", beamBroken());
    SmartDashboard.putNumber("Indexer Current [ST]", indexerMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Has Detected Note [ST]", hasDetectedNote());
  }
  


}
