
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;

public class Shooter extends SubsystemBase {
  // NONE - no motor output, STANDBY - resist gamepiece movement in indexer, HANDOFF - intaking from intake, SPINUP - spinup shooter for speaker scoring
  public static enum ShooterStates {
    NONE,
    STANDBY,
    INTAKE,
    INTAKE_ACCEL, 
    INTAKE_ROLLFORWARD,
    INTAKE_ROLLBACK,
    SPINUP, 
    SCORE_SPEAKER,
    // SCORE_SPEAKER_HIGH,
    SPINUP_SUBWOOFER,
    // SPINUP_PODIUM,
    SPINUP_TEST,
    SCORE_SPEAKER_SUBWOOFER,
    // SCORE_SPEAKER_PODIUM,
    SCORE_SPEAKER_TEST,
    SCORE_AMP,
    SHOOT_CROSSFIELD,
    EJECT_NOTE,

    // JUST IN CASE
    HOLD_NOTE,
    MANUAL
  }
  private ShooterStates state = ShooterStates.NONE;
  private ShooterStates lastState = ShooterStates.NONE;

  // Motors
  TalonFX shooterMotor = new TalonFX(RobotMap.Shooter.SHOOTER_TOP, "canivore");
  TalonFX shooterFollowerMotor = new TalonFX(RobotMap.Shooter.SHOOTER_BOTTOM, "canivore");
  TalonFX indexerMotor = new TalonFX(RobotMap.Shooter.INDEXER, "canivore");
  DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

  PIDController shooterPID = new PIDController(Constants.Shooter.kPShooter, Constants.Shooter.kIShooter, Constants.Shooter.kDShooter);
  PIDController indexerPID = new PIDController(Constants.Shooter.kPIndexer, Constants.Shooter.kIIndexer, Constants.Shooter.kDIndexer);

  // DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

  Supplier<Translation2d> botPoseSupplier;
  // Supplier<Double> distToTagSupplier;
  Supplier<Double> manualOutputAddSupplier;
  Supplier<Double> manualOutputMinusSupplier;
  Supplier<Translation3d> botTranslation3D;

  TorqueCurrentFOC indexerMotorRequest;

  SendableChooser<ShooterStates> stateChooser = new SendableChooser<ShooterStates>();

  double shooterInc = 0.0;
  double indexerMult = 1.0;
  double indexerInc = 0.0;

  // double dist = 0.0;

  public Shooter(Supplier<Translation2d> botPoseSupplier, Supplier<Double> manualOutputAddSupplier, Supplier<Double> manualOutputMinusSupplier, Supplier<Translation3d> botTranslation3D) {
    configMotors();

    shooterMotor.setInverted(Constants.Shooter.kIsShooterTopInverted);
    shooterFollowerMotor.setInverted(Constants.Shooter.kIsShooterBottomInverted);
    indexerMotor.setInverted(Constants.Shooter.kIsIndexerInverted);

    shooterFollowerMotor.setControl(new StrictFollower(shooterMotor.getDeviceID()));
    
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    shooterFollowerMotor.setNeutralMode(NeutralModeValue.Coast);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);

    // initialize torque current FOC request with 0 amps
    this.indexerMotorRequest = new TorqueCurrentFOC(0);
    // mutate request with output of 10 amps and max duty cycle 0.5
  
    
    this.botPoseSupplier = botPoseSupplier;
    // this.distToTagSupplier = distToTagSupplier;
    this.botTranslation3D = botTranslation3D;

    stateChooser.setDefaultOption("NONE", ShooterStates.NONE);
    stateChooser.addOption("STANDBY", ShooterStates.STANDBY);
    stateChooser.addOption("INTAKE", ShooterStates.INTAKE);
    stateChooser.addOption("SPINUP_SUBWOOFER", ShooterStates.SPINUP_SUBWOOFER);
    // stateChooser.addOption("SPINUP_PODIUM", ShooterStates.SPINUP_PODIUM);
    stateChooser.addOption("SPINUP_AMP_ZONE", ShooterStates.SPINUP_TEST);
    stateChooser.addOption("SCORE_SPEAKER_SUBWOOFER", ShooterStates.SCORE_SPEAKER_SUBWOOFER);
    // stateChooser.addOption("SCORE_SPEAKER_PODIUM", ShooterStates.SCORE_SPEAKER_PODIUM);
    stateChooser.addOption("SCORE_SPEAKER_AMP_ZONE", ShooterStates.SCORE_SPEAKER_TEST);
    stateChooser.addOption("SCORE_AMP", ShooterStates.SCORE_AMP);

    SmartDashboard.putData("Shooter State Chooser [ST]", stateChooser);

    ShuffleboardPIDTuner.addSlider("Shooter kP [ST]", 0, 1, Constants.Shooter.kPShooter);
    ShuffleboardPIDTuner.addSlider("Shooter kD [ST]", 0, 1, Constants.Shooter.kDShooter);
    
    ShuffleboardPIDTuner.addSlider("Indexer kP [ST]", 0, 1, Constants.Shooter.kPIndexer);
    ShuffleboardPIDTuner.addSlider("Indexer kD [ST]", 0, 1, Constants.Shooter.kDIndexer);

    ShuffleboardPIDTuner.addSlider("Indexer Output Amps [ST]", 0, 10, 1);
  }

  
  @Override
  public void periodic() {
    postSmartDashboardDebug();

    // dist = distToTagSupplier.get();

    // setState(stateChooser.getSelected());
    // postSmartDashboardDebug();

    // shooterPID.setP(ShuffleboardPIDTuner.get("Shooter kP [ST]"));
    // shooterPID.setD(ShuffleboardPIDTuner.get("Shooter kD [ST]"));

    // indexerPID.setP(ShuffleboardPIDTuner.get("Indexer kP [ST]"));
    // indexerPID.setD(ShuffleboardPIDTuner.get("Indexer kD [ST]"));



    // if (Constants.Shooter.kManualSpeedMultiplier * (manualOutputAddSupplier.get() - manualOutputMinusSupplier.get()) >= Constants.Shooter.kManualIntakeSupplierTolerance) {
    //   setState(ShooterStates.MANUAL);
    // }

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
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        indexerMotor.set(0);
        indexerInc = 0.0;

        shooterInc = 0.0;
        shooterMotor.set(0);
        break;

      case INTAKE:
        if (lastState != ShooterStates.INTAKE) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedIntake) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake

        shooterInc = 0.0;
        shooterMotor.set(shooterInc);

        SmartDashboard.putNumber("Indexer PercentOutput [ST]", Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput));
      break;

      case INTAKE_ACCEL:
        if (lastState != ShooterStates.INTAKE_ACCEL) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedIntake * 0.75) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake

        shooterInc = 0.0;
        shooterMotor.set(shooterInc);

      break;

      case INTAKE_ROLLFORWARD:
        if (lastState != ShooterStates.INTAKE_ROLLFORWARD) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
          indexerInc = 0.0;
          shooterInc = 0.0;
        }
        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedRollforward) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake


        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedRollforward) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;

      case INTAKE_ROLLBACK:
        if (lastState != ShooterStates.INTAKE_ROLLBACK) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
          indexerInc = 0.0;
          shooterInc = 0.0;
        }
        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedRollback) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake


        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedRollback) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;

      case SPINUP:
        if (lastState != ShooterStates.SPINUP) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        indexerMotor.set(0);
        indexerInc = 0.0;

        // if (dist != 0.0) {
          // shooterInc += shooterPID.calculate(getShooterMPS(), Constants.ShootingSetpoints.getValues(dist)[1]) * Constants.Shooter.kShooterIncramentMultiplier;
          shooterInc += shooterPID.calculate(getShooterMPS(), Util.getTargetScaledVelocity(botTranslation3D.get(), Util.getSpeakerTargetBasedOnAllianceColor())) * Constants.Shooter.kShooterIncramentMultiplier;
          shooterMotor.set(shooterInc);
        // } 
      break;

      case SPINUP_SUBWOOFER: 
        if (lastState != ShooterStates.SPINUP_SUBWOOFER) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        indexerMotor.set(0);
        indexerInc = 0.0;

        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer) * Constants.Shooter.kShooterIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;

      case SCORE_SPEAKER:
        if (lastState != ShooterStates.SCORE_SPEAKER) {
          indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        }

        // indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.ShootingSetpoints.getValues(dist)[1]) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer) * Constants.Shooter.kShooterIncramentMultiplier;
        double motorOutput = Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput);
        indexerMotor.set(motorOutput);

        // Spin up shooter
        
        // if (dist != 0.0) {
          // shooterInc += shooterPID.calculate(getShooterMPS(), Constants.ShootingSetpoints.getValues(dist)[1]) * Constants.Shooter.kShooterIncramentMultiplier;
          shooterInc += shooterPID.calculate(getShooterMPS(), Util.getTargetScaledVelocity(botTranslation3D.get(), Util.getSpeakerTargetBasedOnAllianceColor())) * Constants.Shooter.kShooterIncramentMultiplier;
          shooterMotor.set(shooterInc);
        // } 
      break;

      case SCORE_SPEAKER_SUBWOOFER:
        if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
          indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedScoreSpeaker) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput));
        // ShuffleboardPIDTuner.get("Indexer Output Amps [ST]");
        // indexerMotor.setControl(indexerMotorRequest.withOutput(1.0).withMaxAbsDutyCycle(0.5));


        // Spin up shooter
        shooterMotor.set(shooterInc);
      break;

      case SCORE_SPEAKER_TEST:
        if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
          indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedScoreSpeaker) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput));

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        shooterMotor.set(shooterInc);
      break;

      case SCORE_AMP:
        indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreAmp);
        shooterMotor.set(Constants.Shooter.kShooterSpeedScoreAmp);
      break;

      case SHOOT_CROSSFIELD:
        if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
          indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedCrossField) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput));

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedCrossField) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        shooterMotor.set(shooterInc);

      break;

      case EJECT_NOTE:
        indexerMotor.set(Constants.Shooter.kIndexerEjectNoteSpeed);
        shooterMotor.set(Constants.Shooter.kShooterEjectNoteSpeed);
      break;

      case HOLD_NOTE:
        indexerMotor.set(0);
        shooterMotor.set(0);

        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
      break;

      case MANUAL:
        double indexerSpeedManual = manualOutputAddSupplier.get() - manualOutputMinusSupplier.get();
        indexerSpeedManual *= Constants.Shooter.kManualSpeedMultiplier;
        indexerMotor.set(indexerSpeedManual);
      break;

      // <-- delete later -->
      
      // case SCORE_SPEAKER_HIGH:
      //   indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);

      //   // Spin up shooter
      //   shooterPID.setSetpoint(getShooterTargetMPS());
      //   shooterMotor.set(shooterPID.calculate(getShooterMPS()));
      // break;

      // case SPINUP_PODIUM: 
      //   if (lastState != ShooterStates.SPINUP_PODIUM) {
      //     indexerMotor.setNeutralMode(NeutralModeValue.Brake);
      //   }
      //   indexerMotor.set(0);
      //   indexerInc = 0.0;

      //   // Spin up shooter
        
      //   // if (dist != 0.0) {
      //   //   shooterInc += shooterPID.calculate(getShooterMPS(), Constants.ShootingSetpoints.getValues(dist)[1]) * Constants.Shooter.kShooterIncramentMultiplier;
      //   //   shooterMotor.set(shooterInc);
      //   // } 

      //   // for OC
      //   shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerPodium) * Constants.Shooter.kShooterIncramentMultiplier;
      //   shooterMotor.set(shooterInc);
      // break;

      // case SCORE_SPEAKER_PODIUM:
      //   if (lastState != ShooterStates.SCORE_SPEAKER_SUBWOOFER) {
      //     indexerMotor.setNeutralMode(NeutralModeValue.Coast);
      //   }

      //   indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedScoreSpeaker) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
      //   indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput));

      //   // Spin up shooter
      //   shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerPodium) * Constants.Shooter.kShooterIncramentMultiplier;
      //   shooterMotor.set(shooterInc);
      // break;
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
    double mps = rps * (Constants.Shooter.kShooterWheelDiameter * Math.PI) / Constants.Shooter.kShooterGearRatio;
    return mps;
  }

  public double getIndexerMPS() {
    double rps = indexerMotor.getRotorVelocity().getValueAsDouble();
    double mps = rps * (Constants.Shooter.kIndexerWheelDiameter * Math.PI) / Constants.Shooter.kIndexerGearRatio;
    return mps;
  }

  // public double getShooterTargetMPS() {
  //   double dist = Conversions.poseToDistance(botPoseSupplier.get(), Constants.ShootingSetpoints.targetPoint);
  //   double targetMPS = Constants.ShootingSetpoints.getValues(dist)[1];
  //   return targetMPS;
  // }

  // public boolean beamBroken() {
  //   // return !beamBreak.get();
  //   return false;
  // }

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
    indexerMotor.clearStickyFaults();
  }

  public boolean hasDetectedNoteIndexer() {
    return Math.abs(indexerMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kHasNoteCurrentThresholdIndexer; // might have to change threshold
  }

  public boolean hasDetectedNoteShooter() {
    return Math.abs(shooterMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kHasNoteCurrentThresholdShooter;
  }

  public boolean hasNoteRollbackIndexer() {
    return Math.abs(indexerMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kIntakeRollbackCurrentThreshold;
  }

  public boolean beamBroken() {
    return !beamBreak.get();
  }

  private void configMotors() {
    // CurrentLimitsConfigs currentLimitsConfigsShooter = new CurrentLimitsConfigs();
    // currentLimitsConfigsShooter.withSupplyCurrentLimit(Constants.Shooter.kSupplyCurrentLimitShooter);
    // currentLimitsConfigsShooter.withSupplyCurrentLimitEnable(true);
    // CurrentLimitsConfigs currentLimitsConfigsIndexer = new CurrentLimitsConfigs();
    // currentLimitsConfigsIndexer.withSupplyCurrentLimit(Constants.Shooter.kSupplyCurrentLimitIndexer);
    // currentLimitsConfigsIndexer.withSupplyCurrentLimitEnable(true);

    TalonFXConfiguration motorConfigShooter = new TalonFXConfiguration();
    // motorConfigShooter.withCurrentLimits(currentLimitsConfigsShooter);
    TalonFXConfiguration motorConfigIndexer = new TalonFXConfiguration();
    // motorConfigIndexer.withCurrentLimits(currentLimitsConfigsIndexer);

    shooterMotor.getConfigurator().apply(motorConfigShooter);
    indexerMotor.getConfigurator().apply(motorConfigIndexer);
  }

  public void postSmartDashboardDebug() {
    SmartDashboard.putString("State [ST]", state.toString());

    SmartDashboard.putNumber("Shooter MPS [ST]", getShooterMPS());
    SmartDashboard.putNumber("Indexer MPS [ST]", getIndexerMPS());

    SmartDashboard.putNumber("Shooter Target MPS [ST]", shooterPID.getSetpoint());
    SmartDashboard.putNumber("Indexer Target MPS [ST]", indexerPID.getSetpoint());
    
    SmartDashboard.putBoolean("atGoalShooter [ST]", atGoalShooter());
    
    SmartDashboard.putBoolean("Beam Broken [ST]", beamBroken());

    SmartDashboard.putNumber("Shooter PID Output [ST]", shooterInc);
    SmartDashboard.putNumber("Indexer Increment[ST]", indexerInc);

    SmartDashboard.putNumber("Indexer Current Stator [ST]", indexerMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Current Stator [ST]", shooterMotor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putBoolean("Has Detected Note [ST]", beamBroken());
  }
}
