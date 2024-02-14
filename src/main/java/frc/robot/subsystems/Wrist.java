// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.Util;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  WPI_TalonFX motor = new WPI_TalonFX(RobotMap.Wrist.WRIST);

  WristStates state = WristStates.NONE;
  WristStates lastState;

  ProfiledPIDController profiledPIDController;
  // TrapezoidProfile.State goal = new TrapezoidProfile.State();

  Supplier<Double> joystickSupplier;
  Supplier<Translation2d> botPoseSupplier;

  public enum WristStates {
    NONE,
    STOWED,
    HANDOFF,
    SPEAKER,
    SPEAKER_HIGH,
    AMP,
    CLIMB
  }

  public Wrist(Supplier<Double> joystickSupplier, Supplier<Translation2d> botPoseSupplier) {
    TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityDeg, Constants.Wrist.kMaxAccelerationDeg);
    profiledPIDController = new ProfiledPIDController(Constants.Wrist.kPwrist, Constants.Wrist.kIwrist, Constants.Wrist.kDwrist, trapezoidProfileConstraints);
    this.joystickSupplier = joystickSupplier;

    motor.setInverted(Constants.Wrist.kIsWristInverted);
    motor.setNeutralMode(NeutralMode.Brake);
    
    // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));
    // motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
  }

  @Override
  public void periodic() {

    switch(state) { // TODO: figure out target angles

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        // if (lastState != WristStates.STOWED) {
          // wristController.setSetpoint(Constants.Wrist.targetAngle);
          profiledPIDController.setGoal(Constants.Wrist.kTargetAngleStowed);
        // }
        motor.set(profiledPIDController.calculate(getWristAngle()));
        break;

      // case HANDOFF:
      //   // if (lastState != WristStates.HANDOFF) {
      //     // wristController.setSetpoint(Constants.Wrist.targetAngle);
      //     profiledPIDController.setGoal(Constants.Wrist.kTargetAngleHandoff);
      //   // }
      //   motor.set(profiledPIDController.calculate(getWristAngle()));
      //   break;

      case SPEAKER:
        // if (lastState != WristStates.SPEAKER) {
          // wristController.setSetpoint(Constants.Wrist.targetAngle);
          profiledPIDController.setGoal(getWristTargetDeg());
        // }
        motor.set(profiledPIDController.calculate(getWristAngle()));
        break;

      case SPEAKER_HIGH:
        // if (lastState != WristStates.SPEAKER_HIGH) {
          // wristController.setSetpoint(Constants.Wrist.targetAngle);
          profiledPIDController.setGoal(getWristTargetDeg());
        // }
        motor.set(profiledPIDController.calculate(getWristAngle()));
        break;
        
      case AMP:
        // if (lastState != WristStates.AMP) {
          // wristController.setSetpoint(Constants.Wrist.targetAngle);
          profiledPIDController.setGoal(Constants.Wrist.kTargetAngleAmp);
        // }
        motor.set(profiledPIDController.calculate(getWristAngle()));
        break;

      case CLIMB: // TODO: add limit
        double speed = joystickSupplier.get() * Constants.Wrist.kManualMultiplier;
        profiledPIDController.setGoal(profiledPIDController.getGoal().position + (speed));

        double finalspeed = profiledPIDController.calculate(getWristAngle());
        motor.set(finalspeed);
        break;
    }

    clearStickyFaults();
    lastState = state;
  }

  /**
   * TODO: Test CW or CCW signage 
   * @return Angle of the wrist in degrees, CW+, CCW-
   */
  public double getWristAngle() {
    double deg = Conversions.falconToDegrees(motor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
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
    double goal = profiledPIDController.getGoal().position;
    return Util.inRange(getWristAngle(), (goal - Constants.Wrist.kAtGoalTolerance), (goal + Constants.Wrist.kAtGoalTolerance));
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }


}
