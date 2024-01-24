// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Conversions;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  WPI_TalonFX motor = new WPI_TalonFX(Constants.Wrist.motor1);
  WPI_TalonFX motor1 = new WPI_TalonFX(Constants.Wrist.motor2);
  
  WristStates wristStates;
  WristStates lastState;

  ProfiledPIDController turnPID;
  TrapezoidProfile.Constraints trapezoidProfileConstraints;
  TrapezoidProfile.State goal = new TrapezoidProfile.State();

  Supplier<Double> joyvalue;

  public enum WristStates {
    NONE,
    STOWED,
    HANDOFF,
    SPEAKER,
    SPEAKER_HIGH,
    AMP,
    MANUAL
  }

  public Wrist(Supplier<Double> joyvalue) {
    trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityRadPerSecond, Constants.Wrist.kMaxAccelerationRadPerSecSquared);
    turnPID = new ProfiledPIDController(Constants.Wrist.kPwrist, Constants.Wrist.kIwrist, Constants.Wrist.kDwrist, trapezoidProfileConstraints);
    this.joyvalue = joyvalue;

    motor.setInverted(Constants.Wrist.kIsWristInverted);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {

    switch(wristStates) { // TODO: figure out target angles

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        if (lastState != WristStates.STOWED) {
          // turnPID.setSetpoint(Constants.Wrist.targetAngle);
          turnPID.setGoal(goal);
        }
        motor.set(turnPID.calculate(getAngleofMotor()));
        break;

      case HANDOFF:
        if (lastState != WristStates.HANDOFF) {
          // turnPID.setSetpoint(Constants.Wrist.targetAngle);
          turnPID.setGoal(goal);
        }
        motor.set(turnPID.calculate(getAngleofMotor()));
        break;

      case SPEAKER:
        if (lastState != WristStates.SPEAKER) {
          // turnPID.setSetpoint(Constants.Wrist.targetAngle);
          turnPID.setGoal(goal);
        }
        motor.set(turnPID.calculate(getAngleofMotor()));
        break;

      case SPEAKER_HIGH:
        if (lastState != WristStates.SPEAKER_HIGH) {
          // turnPID.setSetpoint(Constants.Wrist.targetAngle);
          turnPID.setGoal(goal);
        }
        motor.set(turnPID.calculate(getAngleofMotor()));
        break;
        
      case AMP:
        if (lastState != WristStates.AMP) {
          // turnPID.setSetpoint(Constants.Wrist.targetAngle);
          turnPID.setGoal(goal);
        }
        motor.set(turnPID.calculate(getAngleofMotor()));
        break;

      case MANUAL: // TODO: add limit
        double speed = joyvalue.get();
        turnPID.setGoal(turnPID.getGoal().position + (Constants.Wrist.motorSpeed * speed));

        double finalspeed = turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint());
        motor.set(finalspeed);
        break;
    }
  }

  public double getAngleofMotor() {

    double deg = Conversions.falconToDegrees(motor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public void setState(WristStates wristStates) {
    this.wristStates = wristStates;
  }
}
