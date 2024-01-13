// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// TODO: merge with main

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.util.Conversions;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  WPI_TalonFX motor = new WPI_TalonFX(Constants.Wrist.motor);
  // TalonFX motor = new TalonFX(Constants.Wrist.motor);
  WristStates wristStates;
  PIDController turnPID;
  double targetAngle;
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

  public Wrist() {
    turnPID = new PIDController(Constants.Wrist.kPwrist, Constants.Wrist.kIwrist, Constants.Wrist.kDwrist);
    turnPID.enableContinuousInput(-180, 180);
  }

  public double getAngleofMotor() { // TODO: make sure angles are correct

    double deg = Conversions.falconToDegrees(motor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public void setWristState(WristStates wristStates) {
    this.wristStates = wristStates;
    double output;

    switch(wristStates) { // TODO: figure out target angles

      case NONE:

        targetAngle = 0;
        output = turnPID.calculate(getAngleofMotor(), targetAngle);
        break;
      case STOWED:

        targetAngle = 0;
        output = turnPID.calculate(getAngleofMotor(), targetAngle);
        break;
      case HANDOFF:

        targetAngle = 0;
        output = turnPID.calculate(getAngleofMotor(), targetAngle);
        break;
      case SPEAKER:

        targetAngle = 0;
        output = turnPID.calculate(getAngleofMotor(), targetAngle);
        break;
      case SPEAKER_HIGH:

        targetAngle = 0;
        output = turnPID.calculate(getAngleofMotor(), targetAngle);
        break;
      case AMP:

        targetAngle = 0;
        output = turnPID.calculate(getAngleofMotor(), targetAngle);
        break;
      case MANUAL:

        manual();
        break;
    }
  }

  public void manual() { // TODO: find deadband + correct speed

    Supplier<Double> joyvalue = () -> RobotContainer.secondary.getRawAxis(1);
    double speed = joyvalue.get();
    double finalspeed;

    if (speed < -0.03) {
      turnPID.setSetpoint(turnPID.getSetpoint() + 0.01);
    }
    else if (speed > 0.03) {
      turnPID.setSetpoint(turnPID.getSetpoint() + 0.01);
    }
    else {
      turnPID.setSetpoint(turnPID.getSetpoint() + 0.01);
    }
    finalspeed = turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint());
    motor.set(finalspeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
