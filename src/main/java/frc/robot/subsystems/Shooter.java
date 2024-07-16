
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
import frc.util.ShuffleboardIO;
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
    SPINUP_SUBWOOFER,
    SPINUP_PODIUM,
    SPINUP_CROSSFIELD,
    // SCORE,
    EJECT_NOTE,
    HOLD_NOTE
  }
  private ShooterStates state = ShooterStates.NONE;
  private ShooterStates lastState = ShooterStates.NONE;

  // Motors
  TalonFX shooterTop = new TalonFX(RobotMap.Shooter.SHOOTER_TOP, "canivore");
  TalonFX shooterBottom = new TalonFX(RobotMap.Shooter.SHOOTER_BOTTOM, "canivore");
  TalonFX indexerMotor = new TalonFX(RobotMap.Shooter.INDEXER, "canivore");
  DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

  PIDController shooterPID = new PIDController(Constants.Shooter.kPShooter, Constants.Shooter.kIShooter, Constants.Shooter.kDShooter);
  PIDController indexerPID = new PIDController(Constants.Shooter.kPIndexer, Constants.Shooter.kIIndexer, Constants.Shooter.kDIndexer);

  // DigitalInput beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK);

  // Supplier<Double> distToTagSupplier;
  Supplier<Translation2d> botPoseSupplier;
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

    shooterTop.setInverted(Constants.Shooter.kIsShooterTopInverted);
    // shooterFollowerMotor.setControl(new StrictFollower(shooterMotor.getDeviceID()));
    shooterBottom.setInverted(Constants.Shooter.kIsShooterBottomInverted);
    indexerMotor.setInverted(Constants.Shooter.kIsIndexerInverted);
    
    shooterTop.setNeutralMode(NeutralModeValue.Coast);
    shooterBottom.setNeutralMode(NeutralModeValue.Coast);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);

    // initialize torque current FOC request with 0 amps, mutate request with output of 10 amps and max duty cycle 0.5
    this.indexerMotorRequest = new TorqueCurrentFOC(0);
  
    // distance to april tag
    // this.distToTagSupplier = distToTagSupplier;

    this.botPoseSupplier = botPoseSupplier;
    this.botTranslation3D = botTranslation3D;

    stateChooser.setDefaultOption("NONE", ShooterStates.NONE);
    stateChooser.addOption("STANDBY", ShooterStates.STANDBY);
    stateChooser.addOption("INTAKE", ShooterStates.INTAKE);
    stateChooser.addOption("SPINUP_SUBWOOFER", ShooterStates.SPINUP_SUBWOOFER);
    stateChooser.addOption("SPINUP_PODIUM", ShooterStates.SPINUP_PODIUM);

    // stateChooser.addOption("SCORE_SPEAKER_SUBWOOFER", ShooterStates.SCORE_SPEAKER_SUBWOOFER);
    // stateChooser.addOption("SCORE_SPEAKER_PODIUM", ShooterStates.SCORE_SPEAKER_PODIUM);
    // stateChooser.addOption("SCORE_SPEAKER_AMP_ZONE", ShooterStates.SCORE_SPEAKER_AMP_ZONE);
    
    SmartDashboard.putData("Shooter State Chooser [ST]", stateChooser);

    ShuffleboardIO.addSlider("Shooter kP [ST]", 0, 1, Constants.Shooter.kPShooter);
    ShuffleboardIO.addSlider("Shooter kD [ST]", 0, 1, Constants.Shooter.kDShooter);
    
    ShuffleboardIO.addSlider("Indexer kP [ST]", 0, 1, Constants.Shooter.kPIndexer);
    ShuffleboardIO.addSlider("Indexer kD [ST]", 0, 1, Constants.Shooter.kDIndexer);

    // ShuffleboardIO.addSlider("Indexer Output Amps [ST]", 0, 10, 1);

    ShuffleboardIO.addSlider("kShooterSpeedScoreAmp [ST]", 0, 1.0, Constants.Shooter.kShooterSpeedScoreAmp);
  }

  
  @Override
  public void periodic() {
    postSmartDashboardDebug();
    
    switch (state) {

      case NONE:
        setShooterSpeed(0);
        indexerMotor.set(0);
      break;

      case STANDBY:


        if (lastState != ShooterStates.STANDBY) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        indexerMotor.set(0);
        indexerInc = 0.0;

        shooterInc = 0.0;
        setShooterSpeed(0);
        break;

      case INTAKE:
        if (lastState != ShooterStates.INTAKE) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedIntake) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake

        shooterInc = 0.0;
        setShooterSpeed(shooterInc);

        // debug
        // SmartDashboard.putNumber("Indexer PercentOutput [ST]", Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput));
      
        break;

      case INTAKE_ACCEL:
        if (lastState != ShooterStates.INTAKE_ACCEL) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedIntake * 0.75) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake

        shooterInc = 0.0;
        setShooterSpeed(shooterInc);

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
        setShooterSpeed(shooterInc);
      break;

      case INTAKE_ROLLBACK:
        if (lastState != ShooterStates.INTAKE_ROLLBACK) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
          indexerInc = 0.0;
          shooterInc = 0.0;
        }
        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedRollback) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake

        setShooterSpeed(Constants.Shooter.kShooterSpeedRollbackPercent);
      break;

      case SPINUP_SUBWOOFER: 
        //distance shooter
        // if (lastState != ShooterStates.SPINUP_SUBWOOFER) {
        //   indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        // }
        // indexerMotor.set(0);
        // indexerInc = 0.0;

        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer) * Constants.Shooter.kShooterIncramentMultiplier;
        setShooterSpeed(shooterInc);

        SmartDashboard.putNumber("Shooter PercentOutput [ST]", shooterInc);
      break;

      case SPINUP_PODIUM: 
        //distance shooter
        // if (lastState != ShooterStates.SPINUP_PODIUM) {
        //   indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        // }
        // indexerMotor.set(0);
        // indexerInc = 0.0;

        // Spin up shooter
        
        // if (dist != 0.0) {
        //   shooterInc += shooterPID.calculate(getShooterMPS(), Constants.ShootingSetpoints.getValues(dist)[1]) * Constants.Shooter.kShooterIncramentMultiplier;
        //   shooterMotor.set(shooterInc);
        // } 

        // for OC
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedScoreSpeakerPodium) * Constants.Shooter.kShooterIncramentMultiplier;
        setShooterSpeed(shooterInc);
      break;

      case SPINUP_CROSSFIELD:
        // if (lastState != ShooterStates.SPINUP_CROSSFIELD) {
        //   indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        // }

        // indexerMotor.set(0.0);
        // indexerInc = 0.0;

        // Spin up shooter
        shooterInc += shooterPID.calculate(getShooterMPS(), Constants.Shooter.kShooterSpeedCrossField) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        setShooterSpeed(shooterInc);

      break;

      // case SCORE:
      // break;

      case EJECT_NOTE:
        indexerMotor.set(Constants.Shooter.kIndexerEjectNoteSpeed);
        setShooterSpeed(Constants.Shooter.kShooterEjectNoteSpeed);
      break;

      case HOLD_NOTE:
        indexerMotor.set(0);
        setShooterSpeed(0);

        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterTop.setNeutralMode(NeutralModeValue.Brake);
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

  public void setIndexerSpeedScoreSpeaker() {
    indexerMotor.set(Constants.Shooter.kIndexerSpeedScoreSpeaker);
  }

  public void setIndexerRollBack() {
    indexerMotor.set(Constants.Shooter.kIndexerSpeedRollback);
  }

  public double getShooterMPS() {
    double rps = shooterTop.getRotorVelocity().getValueAsDouble();
    double mps = rps * (Constants.Shooter.kShooterWheelDiameter * Math.PI) / Constants.Shooter.kShooterGearRatio;
    return mps;
  }

  public double getIndexerMPS() {
    double rps = indexerMotor.getRotorVelocity().getValueAsDouble();
    double mps = rps * (Constants.Shooter.kIndexerWheelDiameter * Math.PI) / Constants.Shooter.kIndexerGearRatio;
    return mps;
  }

  public boolean atGoalShooter() {
    double goal = shooterPID.getSetpoint();
    boolean inRange = Util.inRange(getShooterMPS(), (goal - Constants.Shooter.kAtGoalTolerance), (goal + Constants.Shooter.kAtGoalTolerance));
    return inRange;
  }

  public void clearStickyFaults() {
    shooterTop.clearStickyFaults();
    shooterBottom.clearStickyFaults();
    indexerMotor.clearStickyFaults();
  }

  public boolean hasDetectedNoteIndexer() {
    return Math.abs(indexerMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kHasNoteCurrentThresholdIndexer; // might have to change threshold
  }

  public boolean hasDetectedNoteShooter() {
    return Math.abs(shooterTop.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kHasNoteCurrentThresholdShooter;
  }

  public boolean beamBroken() {
    return !beamBreak.get();
  }

  public boolean hasNoteRollbackIndexer() {
    return (Math.abs(indexerMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kIntakeRollbackCurrentThresholdIndexer) && beamBroken();
  }

  public boolean hasNoteRollbackShooter() {
    return Math.abs(shooterTop.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kIntakeRollbackCurrentThresholdShooter;
  }

  public void setShooterSpeed(double speed) {
    shooterTop.set(speed);
    shooterBottom.set(speed * Constants.Shooter.kShooterBottomSpeedMultiplier);
  }

  public void zeroPosition() {
    shooterTop.setPosition(0);
    shooterBottom.setPosition(0);
    indexerMotor.setPosition(0);
  }

  private void configMotors() {

    // current configs

    // CurrentLimitsConfigs currentLimitsConfigsShooter = new CurrentLimitsConfigs();
    // currentLimitsConfigsShooter.withSupplyCurrentLimit(Constants.Shooter.kSupplyCurrentLimitShooter);
    // currentLimitsConfigsShooter.withSupplyCurrentLimitEnable(true);
    // CurrentLimitsConfigs currentLimitsConfigsIndexer = new CurrentLimitsConfigs();
    // currentLimitsConfigsIndexer.withSupplyCurrentLimit(Constants.Shooter.kSupplyCurrentLimitIndexer);
    // currentLimitsConfigsIndexer.withSupplyCurrentLimitEnable(true);

    TalonFXConfiguration motorConfigShooter = new TalonFXConfiguration();
    TalonFXConfiguration motorConfigIndexer = new TalonFXConfiguration();

    // current limiting
    // motorConfigShooter.withCurrentLimits(currentLimitsConfigsShooter);
    // motorConfigIndexer.withCurrentLimits(currentLimitsConfigsIndexer);

    shooterTop.getConfigurator().apply(motorConfigShooter);
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
    SmartDashboard.putNumber("Shooter Current Stator [ST]", shooterTop.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Indexer Current Supply [ST]", indexerMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Current Supply [ST]", shooterTop.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putBoolean("Has Detected Note [ST]", beamBroken());
    SmartDashboard.putBoolean("Has Detected Note Rollback [ST]", hasNoteRollbackIndexer());
  }
}
