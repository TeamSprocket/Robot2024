
package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  public static enum ShooterStates {
    NONE,
    STANDBY,
    INTAKE,
    SPINUP_SUBWOOFER,
    SPINUP_PODIUM,
    SPINUP_CROSSFIELD,
    EJECT_NOTE
  }
  private ShooterStates state = ShooterStates.NONE;
  private ShooterStates lastState = ShooterStates.NONE;

  TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  VelocityVoltage sVV = new VelocityVoltage(0);

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

  public Shooter(Supplier<Translation2d> botPoseSupplier, Supplier<Double> manualOutputAddSupplier, Supplier<Double> manualOutputMinusSupplier, Supplier<Translation3d> botTranslation3D) {
    Slot0Configs shooterSlot0 = new Slot0Configs()
      .withKS(0.22)
      .withKV(0.080) //0.095
      .withKA(0.001)
      .withKP(0.2)
      .withKD(0);
    FeedbackConfigs shooterFeedback = new FeedbackConfigs().withSensorToMechanismRatio(Constants.Shooter.kShooterGearRatio);
    shooterConfig.withSlot0(shooterSlot0); shooterConfig.withFeedback(shooterFeedback);

    shooterTop.setInverted(Constants.Shooter.kIsShooterTopInverted);
    shooterBottom.setInverted(Constants.Shooter.kIsShooterBottomInverted);
    indexerMotor.setInverted(Constants.Shooter.kIsIndexerInverted);
    
    shooterTop.setNeutralMode(NeutralModeValue.Coast);
    shooterBottom.setNeutralMode(NeutralModeValue.Coast);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);

    shooterTop.getConfigurator().apply(shooterConfig);
    shooterBottom.getConfigurator().apply(shooterConfig);
    shooterBottom.setControl(new StrictFollower(17));

    // initialize torque current FOC request with 0 amps, mutate request with output of 10 amps and max duty cycle 0.5
    this.indexerMotorRequest = new TorqueCurrentFOC(0);
  
    // distance to april tag

    this.botPoseSupplier = botPoseSupplier;
    this.botTranslation3D = botTranslation3D;

    stateChooser.setDefaultOption("NONE", ShooterStates.NONE);
    stateChooser.addOption("STANDBY", ShooterStates.STANDBY);
    stateChooser.addOption("INTAKE", ShooterStates.INTAKE);
    stateChooser.addOption("SPINUP_SUBWOOFER", ShooterStates.SPINUP_SUBWOOFER);
    stateChooser.addOption("SPINUP_PODIUM", ShooterStates.SPINUP_PODIUM);

    SmartDashboard.putData("Shooter State Chooser [ST]", stateChooser);

    ShuffleboardIO.addSlider("Shooter kP [ST]", 0, 1, Constants.Shooter.kPShooter);
    ShuffleboardIO.addSlider("Shooter kD [ST]", 0, 1, Constants.Shooter.kDShooter);
    
    ShuffleboardIO.addSlider("Indexer kP [ST]", 0, 1, Constants.Shooter.kPIndexer);
    ShuffleboardIO.addSlider("Indexer kD [ST]", 0, 1, Constants.Shooter.kDIndexer);

    ShuffleboardIO.addSlider("kShooterSpeedScoreAmp [ST]", 0, 1.0, Constants.Shooter.kShooterSpeedScoreAmp);
  }

  
  @Override
  public void periodic() {
    postSmartDashboardDebug();

    switch (state) {

      case NONE:
        shooterTop.setControl(sVV.withVelocity(0));
        indexerMotor.set(0);
      break;

      case STANDBY:


        if (lastState != ShooterStates.STANDBY) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        indexerMotor.set(0);
        indexerInc = 0.0;

        shooterTop.setControl(sVV.withVelocity(0));
        break;

      case INTAKE:
        if (lastState != ShooterStates.INTAKE) {
          indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        }

        indexerInc += indexerPID.calculate(getIndexerMPS(), Constants.Shooter.kIndexerSpeedIntake) * Constants.Shooter.kShooterkIndexerIncramentMultiplier;
        indexerMotor.set(Util.minmax(indexerInc, -1 * Constants.Shooter.kMaxIndexerOutput, Constants.Shooter.kMaxIndexerOutput)); // TODO: remove pid from intake

        shooterTop.setControl(sVV.withVelocity(0));

        break;

      case SPINUP_SUBWOOFER: 

        shooterTop.setControl(sVV.withVelocity(Constants.Shooter.kShooterSpeedScoreSpeakerSubwoofer));

        SmartDashboard.putNumber("Shooter PercentOutput [ST]", shooterInc);
      break;

      case SPINUP_PODIUM: 
        shooterTop.setControl(sVV.withVelocity(Constants.Shooter.kShooterSpeedScoreSpeakerPodium));
      break;

      case SPINUP_CROSSFIELD:
        shooterTop.setControl(sVV.withVelocity(Constants.Shooter.kShooterSpeedCrossField));

      break;

      case EJECT_NOTE:
        indexerMotor.set(Constants.Shooter.kIndexerEjectNoteSpeed);
        shooterTop.setControl(sVV.withVelocity(Constants.Shooter.kShooterEjectNoteSpeed));
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

  // possibly useful methods
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

  public boolean hasNoteRollbackIndexer() {
    return Math.abs(indexerMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kIntakeRollbackCurrentThresholdIndexer;
  }

  public boolean hasNoteRollbackShooter() {
    return Math.abs(shooterTop.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.kIntakeRollbackCurrentThresholdShooter;
  }

  public boolean beamBroken() {
    return !beamBreak.get();
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
  }
}
