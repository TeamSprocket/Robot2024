// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */
  TalonFX motor = new TalonFX(RobotMap.ShooterPivot.WRIST);

  ShooterPivotStates state = ShooterPivotStates.NONE;
  ShooterPivotStates lastState = ShooterPivotStates.NONE;

  double motorspeed = 0.0;

  // ProfiledpidController profiledpidController;
  // TrapezoidProfile.State goal = new TrapezoidProfile.State();
  PIDController pidController = new PIDController(Constants.ShooterPivot.kPID.kP, Constants.ShooterPivot.kPID.kI, Constants.ShooterPivot.kPID.kD);

  Supplier<Double> joystickSupplier;
  Supplier<Double> distToTagSupplier;

  SendableChooser<ShooterPivotStates> selectShooterPivotState = new SendableChooser<ShooterPivotStates>();

  public enum ShooterPivotStates {
    NONE,
    STOWED,
    INTAKE,
    SPEAKER_SUBWOOFER, SPEAKER_PODIUM, SPEAKER_AMP_ZONE,
    SPEAKER,
    // SPEAKER_HIGH,
    AMP,
    CLIMB,
    SOURCE
  }

  public ShooterPivot(Supplier<Double> joystickSupplier, Supplier<Double> distToTagSupplier) {
    // TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.ShooterPivot.kMaxVelocityDeg, Constants.ShooterPivot.kMaxAccelerationDeg);
    // profiledpidController = new ProfiledpidController(Constants.ShooterPivot.kPshooterPivot, Constants.ShooterPivot.kIshooterPivot, Constants.ShooterPivot.kDshooterPivot, trapezoidProfileConstraints);
    this.joystickSupplier = joystickSupplier;
    this.distToTagSupplier = distToTagSupplier;

    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30)); // test values later
    
    selectShooterPivotState.setDefaultOption("NONE", ShooterPivotStates.NONE);
    selectShooterPivotState.addOption("STOWED", ShooterPivotStates.STOWED);
    selectShooterPivotState.addOption("INTAKE", ShooterPivotStates.INTAKE);
    selectShooterPivotState.addOption("SPEAKER", ShooterPivotStates.SPEAKER);
    // selectShooterPivotState.addOption("SPEAKER_HIGH", ShooterPivotStates.SPEAKER_HIGH);
    selectShooterPivotState.addOption("SPEAKER AMP ZONE", ShooterPivotStates.SPEAKER_AMP_ZONE);
    selectShooterPivotState.addOption("SPEAKER PODIUM", ShooterPivotStates.SPEAKER_PODIUM);
    selectShooterPivotState.addOption("SPEAKER SUBWOOFER", ShooterPivotStates.SPEAKER_SUBWOOFER);
    selectShooterPivotState.addOption("AMP", ShooterPivotStates.AMP);
    selectShooterPivotState.addOption("CLIMB", ShooterPivotStates.CLIMB);
    SmartDashboard.putData("State Selector [SP]", selectShooterPivotState);

    SmartDashboard.putNumber("shooterPivot angle", getShooterPivotAngle());
    SmartDashboard.putNumber("shooterPivot velocity", motor.getVelocity().getValueAsDouble());

    // ShuffleboardPIDTuner.addSlider("shooterPivot kP", 0.0, 1, Constants.ShooterPivot.kPID.kP);
    // ShuffleboardPIDTuner.addSlider("shooterPivot kD", 0.0, 0.05, 0.0);
  }

  @Override
  public void periodic() {
    // pidController.setP(ShuffleboardPIDTuner.get("shooterPivot kP"));
    // pidController.setD(ShuffleboardPIDTuner.get("shooterPivot kD"));

    SmartDashboard.putString("PivotState", state.toString());
    setState(selectShooterPivotState.getSelected());

    switch(state) { // TODO: figure out target angles

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleStowed);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case INTAKE:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleIntake);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER:
        double dist = distToTagSupplier.get();
        // System.out.println(dist);
        if (dist != 0.0) {
          double angleTarget = Constants.ShootingSetpoints.getValues(dist)[0];
          double angleTargetAdjusted = Constants.ShooterPivot.kHorizontalAngle - angleTarget;

          pidController.setSetpoint(angleTargetAdjusted);

          SmartDashboard.putNumber("Target Angle MECHANISM [SP]", angleTarget);
          SmartDashboard.putNumber("Target Angle ADJUSTED [SP]", angleTargetAdjusted);
        }
        SmartDashboard.putNumber("Pivot Distance [SP]", dist);
        
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_AMP_ZONE:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleSpeakerFromAmp);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_PODIUM:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleSpeakerFromPodium);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_SUBWOOFER:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;
        
      case AMP:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleAmp);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      // JUST IN CASE

      case SOURCE:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleSource);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      //

      case CLIMB: // TODO: add limit
        double speed = joystickSupplier.get() * Constants.ShooterPivot.kManualMultiplier;
        pidController.setSetpoint(pidController.getSetpoint() + (speed));

        double finalspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        finalspeed = Util.minmax(finalspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(finalspeed);
        break;
    }

    // clearStickyFaults();
    lastState = state;
    SmartDashboard.putNumber("ShooterPivot Angle [SP]", getShooterPivotAngle());
  }

  /**
   * TODO: Test CW or CCW signage 
   * @return Angle of the shooterPivot in degrees, CW+, CCW-
   */
  public double getShooterPivotAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.ShooterPivot.kShooterPivotGearRatio);
    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public void setState(ShooterPivotStates state) {
    this.state = state;
  }

  public ShooterPivotStates getState() {
      return state;
  }

  // public double getShooterPivotTargetDeg() {
  //   double dist = Conversions.poseToDistance(botPoseSupplier.get(), Constants.ShootingSetpoints.targetPoint);
  //   double targetDeg = Constants.ShootingSetpoints.getValues(dist)[0];
  //   return targetDeg;
  // }

  public void debug() {
    SmartDashboard.putNumber("Angle in Degrees", getShooterPivotAngle());
  }

  public boolean atGoal() {
    double goal = pidController.getSetpoint();
    return Util.inRange(getShooterPivotAngle(), (goal - Constants.ShooterPivot.kAtGoalTolerance), (goal + Constants.ShooterPivot.kAtGoalTolerance));
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }


}
