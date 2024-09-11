// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardIO;
import frc.util.Util;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */
  TalonFX motor = new TalonFX(RobotMap.ShooterPivot.WRIST, "canivore");

  double motorspeed = 0.0;
  
  Supplier<Double> joystickSupplier;
  Supplier<Translation3d> botPoseSupplier;


  SendableChooser<ShooterPivotStates> selectShooterPivotState = new SendableChooser<ShooterPivotStates>();

  /**
   * Individual shooterPivot states; for each state, different things are activated to satisfy each state.
   */
  public enum ShooterPivotStates {
    NONE,
    STOWED,
    INTAKE,
    INDEXING,
    EJECT_NOTE,
    SPEAKER_PODIUM,
    SPEAKER_AMP_ZONE,
    SPEAKER_SUBWOOFER,
    SPEAKER,
    AMP,
    CROSSFIELD,
    CLIMB  
  }

  ShooterPivotStates state = ShooterPivotStates.NONE;
  ShooterPivotStates lastState = ShooterPivotStates.NONE;

  private MotionMagicVoltage motionMagicVolt;

  /**
   * Constructor for shooterPivot
   * @param joystickSupplier
   * @param botPoseSupplier
   */
  public ShooterPivot(Supplier<Double> joystickSupplier, Supplier<Translation3d> botPoseSupplier) {

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.withMotionMagic(new MotionMagicConfigs().withMotionMagicCruiseVelocity(Constants.ShooterPivot.KMotionMagicCruiseVelocity).withMotionMagicAcceleration(Constants.ShooterPivot.KMotionMagicAcceleration)); // TODO: tune velocity and acceleration for motion magic
    motorConfig.withSlot0(new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.ShooterPivot.KShooterPivotKS) 
                .withKV(Constants.ShooterPivot.KShooterPivotKV) 
                .withKA(Constants.ShooterPivot.KShooterPivotKA) 
                .withKG(Constants.ShooterPivot.KShooterPivotKG) 
                .withKP(Constants.ShooterPivot.KShooterPivotKP)
                .withKI(Constants.ShooterPivot.KShooterPivotKI)
                .withKD(Constants.ShooterPivot.KShooterPivotKD));

    motorConfig.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterPivot.kShooterPivotGearRatio)
        );

    motor.getConfigurator().apply(motorConfig);
    // motorConfig.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.ShooterPivot.kStatorCurrentLimit).withStatorCurrentLimitEnable(true));

    //init global variable 
    motionMagicVolt = new MotionMagicVoltage(0);

    this.joystickSupplier = joystickSupplier;
    this.botPoseSupplier = botPoseSupplier;

    //configures the motors
    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    
    motor.setPosition(-0.145);

    //Adds options for the selection thing
    selectShooterPivotState.setDefaultOption("NONE", ShooterPivotStates.NONE);
    selectShooterPivotState.addOption("STOWED", ShooterPivotStates.STOWED);
    selectShooterPivotState.addOption("INTAKE", ShooterPivotStates.INTAKE);
    selectShooterPivotState.addOption("SPEAKER", ShooterPivotStates.SPEAKER);
    selectShooterPivotState.addOption("SPEAKER AMP ZONE", ShooterPivotStates.SPEAKER_AMP_ZONE);
    selectShooterPivotState.addOption("SPEAKER PODIUM", ShooterPivotStates.SPEAKER_PODIUM);
    selectShooterPivotState.addOption("AMP", ShooterPivotStates.AMP);
    selectShooterPivotState.addOption("CLIMB", ShooterPivotStates.CLIMB);

    SmartDashboard.putData("State Selector [SP]", selectShooterPivotState);

    ShuffleboardIO.addSlider("kHorizontalAngle ShooterPivot [SP]", 5.0, 90.0, Constants.ShooterPivot.kTargetAngleStowed);
    ShuffleboardIO.addSlider("kTargetAngleAmp [SP]", 60.0, 120, Constants.ShooterPivot.kTargetAngleAmp);
  }

  @Override
  public void periodic() {
    debug();
    motionmagicPeriodic();
  }

  /**
   * @return Angle of the shooterPivot in radians, CW+, CCW-
   */

  public double getShooterPivotAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.ShooterPivot.kShooterPivotGearRatio);

    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }

    return deg;
  }

  /**
   * Gets the pivot speed
   * @param targetAngle Uses the targetAngle to get a set pivot speed.
   * @return
   */

  public void setState(ShooterPivotStates state) {
    this.state = state;
  }

  public ShooterPivotStates getState() {
      return state;
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    motor.setNeutralMode(neutralModeValue);
  }


  public void debug() {
    SmartDashboard.putNumber("Angle in Degrees [SP]", getShooterPivotAngle());
    SmartDashboard.putNumber("Shot Target Angle [SP]", Util.getTargetShotAngleDeg(botPoseSupplier.get(), Util.getSpeakerTargetBasedOnAllianceColor()));
    SmartDashboard.putNumber("Position [SP]", motor.getPosition().getValueAsDouble());
    SmartDashboard.putString("State [SP]", state.toString());
  }

  public void zeroPosition() {
    motor.setPosition(0);
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }


  /**call this function to run periodic with motion magic */
  private void motionmagicPeriodic() {
    //init default to put in dashboard
    switch (state) {
      case NONE:
      motor.set(0);
        break;
    
       case STOWED:

        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleStowed));
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case EJECT_NOTE:

        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleEject));
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;


      case INTAKE:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleIntake));
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case INDEXING:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleIndexing));    

        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;
    
      //may not work
      case SPEAKER:
        Translation3d botPose = botPoseSupplier.get();
        System.out.println(botPose.getX() + botPose.getY() + botPose.getZ());
        if (botPose.getX() + botPose.getY() + botPose.getZ() != 0.0) {

          double angleTarget = Util.getTargetShotAngleDeg(botPose, Util.getSpeakerTargetBasedOnAllianceColor());
          double angleTargetAdjusted = Constants.ShooterPivot.kHorizontalAngle - angleTarget;

          if (angleTargetAdjusted < Constants.ShooterPivot.kTargetAngleStowed || angleTargetAdjusted > Constants.ShooterPivot.kTargetAngleAmp) {
            angleTargetAdjusted = Constants.ShooterPivot.kTargetAngleStowed;
          }

          motor.setControl(motionMagicVolt.withPosition(angleTargetAdjusted));  

          SmartDashboard.putNumber("Target Angle MECHANISM [SP]", angleTarget);
          SmartDashboard.putNumber("Target Angle ADJUSTED [SP]", angleTargetAdjusted);
        } else {
          SmartDashboard.putNumber("Target Angle MECHANISM [SP]", -1.0);
          SmartDashboard.putNumber("Target Angle ADJUSTED [SP]", -1.0);
          motor.set(0);
        }        
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case SPEAKER_SUBWOOFER:

        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer));        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
      break;

      case SPEAKER_AMP_ZONE: 
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleSpeakerFromAmpZone));        
        break;

      case SPEAKER_PODIUM:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAnglePodium));        

        break;
 
      case CROSSFIELD:

        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleCrossfield));        
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);

        break;
        
      case AMP:
        
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleAmp));        
        
        break;

      case CLIMB: // TODO: FIX LOL
      //   double speed = joystickSupplier.get() * Constants.ShooterPivot.kManualMultiplier;
      //   motionMagicVolt = motorMotionMagicController.withPosition( + (speed));

      // motor.setControl(motionMagicVolt);  
        break;

  }
}
}
