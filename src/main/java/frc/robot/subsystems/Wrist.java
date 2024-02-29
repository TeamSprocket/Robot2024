// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.pidController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
// import frc.robot.subsystems.Intake.IntakeStates;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  TalonFX motor = new TalonFX(RobotMap.Wrist.WRIST);

  WristStates state = WristStates.NONE;
  WristStates lastState;

  double motorspeed = 0.0;

  // ProfiledpidController profiledpidController;
  // TrapezoidProfile.State goal = new TrapezoidProfile.State();
  PIDController pidController = new PIDController(Constants.Wrist.kPwrist, Constants.Wrist.kIwrist, Constants.Wrist.kDwrist);

  Supplier<Double> joystickSupplier;
  Supplier<Translation2d> botPoseSupplier;

  SendableChooser<WristStates> selectWristState = new SendableChooser<WristStates>();

  public enum WristStates {
    NONE,
    STOWED,
    INTAKE,
    SPEAKER_SUBWOOFER, SPEAKER_PODIUM, SPEAKER_AMP_ZONE,
    // SPEAKER,
    // SPEAKER_HIGH,
    AMP,
    CLIMB,
    SOURCE
  }

  public Wrist(Supplier<Double> joystickSupplier, Supplier<Translation2d> botPoseSupplier) {
    // TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityDeg, Constants.Wrist.kMaxAccelerationDeg);
    // profiledpidController = new ProfiledpidController(Constants.Wrist.kPwrist, Constants.Wrist.kIwrist, Constants.Wrist.kDwrist, trapezoidProfileConstraints);
    this.joystickSupplier = joystickSupplier;

    motor.setInverted(Constants.Wrist.kIsWristInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    
    // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));
    // motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
    selectWristState.setDefaultOption("NONE", WristStates.NONE);
    selectWristState.addOption("STOWED", WristStates.STOWED);
    selectWristState.addOption("INTAKE", WristStates.INTAKE);
    // selectWristState.addOption("SPEAKER", WristStates.SPEAKER);
    // selectWristState.addOption("SPEAKER_HIGH", WristStates.SPEAKER_HIGH);
    selectWristState.addOption("SPEAKER AMP ZONE", WristStates.SPEAKER_AMP_ZONE);
    selectWristState.addOption("SPEAKER PODIUM", WristStates.SPEAKER_PODIUM);
    selectWristState.addOption("SPEAKER SUBWOOFER", WristStates.SPEAKER_SUBWOOFER);
    selectWristState.addOption("AMP", WristStates.AMP);
    selectWristState.addOption("CLIMB", WristStates.CLIMB);

    SmartDashboard.putData(selectWristState);
    SmartDashboard.putNumber("wrist angle", getWristAngle());
    SmartDashboard.putNumber("wrist velocity", motor.getVelocity().getValueAsDouble());

    ShuffleboardPIDTuner.addSlider("wrist kP", 0.0, 1, Constants.Wrist.kPwrist);
    // ShuffleboardPIDTuner.addSlider("wrist kD", 0.0, 0.05, 0.0);
  }

  @Override
  public void periodic() {
    pidController.setP(ShuffleboardPIDTuner.get("wrist kP"));
    // pidController.setD(ShuffleboardPIDTuner.get("wrist kD"));

    setState(selectWristState.getSelected());

    switch(state) { // TODO: figure out target angles

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleStowed);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;

      case INTAKE:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleIntake);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_AMP_ZONE:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleSpeakerFromAmp);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_PODIUM:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleSpeakerFromPodium);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_SUBWOOFER:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleSpeakerFromSubwoofer);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;
        
      case AMP:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleAmp);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;

      // JUST IN CASE

      case SOURCE:
        pidController.setSetpoint(Constants.Wrist.kTargetAngleSource);
        motorspeed = pidController.calculate(getWristAngle());

        motorspeed = Util.minmax(motorspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(motorspeed);
        break;

      //

      case CLIMB: // TODO: add limit
        double speed = joystickSupplier.get() * Constants.Wrist.kManualMultiplier;
        pidController.setSetpoint(pidController.getSetpoint() + (speed));

        double finalspeed = pidController.calculate(getWristAngle());

        finalspeed = Util.minmax(finalspeed, -1 * Constants.Wrist.kMaxWristOutput, Constants.Wrist.kMaxWristOutput);
        motor.set(finalspeed);
        break;
    }

    // clearStickyFaults();
    lastState = state;
    SmartDashboard.putNumber("Wrist Angle [WR]", getWristAngle());
  }

  /**
   * TODO: Test CW or CCW signage 
   * @return Angle of the wrist in degrees, CW+, CCW-
   */
  public double getWristAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.Wrist.kWristGearRatio);
    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public void setState(WristStates state) {
    this.state = state;
  }

  public WristStates getState() {
      return state;
  }

  public double getWristTargetDeg() {
    double dist = Conversions.poseToDistance(botPoseSupplier.get(), Constants.ShootingSetpoints.targetPoint);
    double targetDeg = Constants.ShootingSetpoints.getValues(dist)[0];
    return targetDeg;
  }

  public void debug() {
    SmartDashboard.putNumber("Angle in Degrees", getWristAngle());
  }

  public boolean atGoal() {
    double goal = pidController.getSetpoint();
    return Util.inRange(getWristAngle(), (goal - Constants.Wrist.kAtGoalTolerance), (goal + Constants.Wrist.kAtGoalTolerance));
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }


}
