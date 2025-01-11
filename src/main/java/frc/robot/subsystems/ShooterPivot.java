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
import frc.util.Util;

public class ShooterPivot extends SubsystemBase {
/** Creates a new ShooterPivot. */
  TalonFX motor = new TalonFX(RobotMap.ShooterPivot.WRIST);

  double motorspeed = 0.0;

  Supplier<Double> joystickSupplier;
  // Supplier<Translation3d> botPoseSupplier;

  SendableChooser<ShooterPivotStates> selectShooterPivotState = new SendableChooser<ShooterPivotStates>();

  /**
   * Individual shooterPivot states; for each state, different things are activated to satisfy each state.
   */
  public enum ShooterPivotStates {
    NONE,
    STOWED,
    INTAKE,
    EJECT_NOTE,
    SPEAKER_PODIUM,
    SPEAKER_SUBWOOFER,
    AMP,
    CROSSFIELD
  }

  ShooterPivotStates state = ShooterPivotStates.NONE;
  ShooterPivotStates lastState = ShooterPivotStates.NONE;

  private MotionMagicVoltage motionMagicVolt;

  /**
   * Constructor for shooterPivot
   * @param joystickSupplier
   * @param botPoseSupplier
   */
  public ShooterPivot() {

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

    //init global variable 
    motionMagicVolt = new MotionMagicVoltage(0);

    // this.joystickSupplier = joystickSupplier;
    // this.botPoseSupplier = botPoseSupplier;

    //configures the motors
    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    
    motor.setPosition(-0.145);

    //Adds options for the selection thing
    selectShooterPivotState.setDefaultOption("NONE", ShooterPivotStates.NONE);
    selectShooterPivotState.addOption("STOWED", ShooterPivotStates.STOWED);
    selectShooterPivotState.addOption("INTAKE", ShooterPivotStates.INTAKE);
    selectShooterPivotState.addOption("SPEAKER PODIUM", ShooterPivotStates.SPEAKER_PODIUM);
    selectShooterPivotState.addOption("AMP", ShooterPivotStates.AMP);

    SmartDashboard.putData("State Selector [SP]", selectShooterPivotState);
  }

  @Override
  public void periodic() {
    debug();
    
    switch (state) {
      case NONE:
        motor.set(0);
      break;
      
      case STOWED:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleStowed));
      break;

      case EJECT_NOTE:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleEject));        
      break;

      case INTAKE:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleIntake));        
      break;

      case SPEAKER_SUBWOOFER:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer));        
      break;

      case SPEAKER_PODIUM:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAnglePodium));        
      break;

      case CROSSFIELD:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleCrossfield));        
      break;

      case AMP:
        motor.setControl(motionMagicVolt.withPosition(Constants.ShooterPivot.kTargetAngleAmp));          
      break;
      }
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
    // SmartDashboard.putNumber("Shot Target Angle [SP]", Util.getTargetShotAngleDeg(botPoseSupplier.get(), Util.getSpeakerTargetBasedOnAllianceColor()));
    SmartDashboard.putNumber("Position [SP]", motor.getPosition().getValueAsDouble());
    SmartDashboard.putString("State [SP]", state.toString());
  }

  public void zeroPosition() {
    motor.setPosition(0);
  }

  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }
}
