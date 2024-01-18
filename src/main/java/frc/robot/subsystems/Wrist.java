// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Conversions;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  WPI_TalonFX motor = new WPI_TalonFX(Constants.Wrist.motor);
  // TalonFX motor = new TalonFX(Constants.Wrist.motor);
  WristStates wristStates; // Just call this state, put on line 36 for organization 
  PIDController turnPID;
  double targetAngle; // prob dont need, just set pid setpoint directly
  Supplier<Double> joyvalue;
  double output;

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
    turnPID = new PIDController(Constants.Wrist.kPwrist, Constants.Wrist.kIwrist, Constants.Wrist.kDwrist); // capitalized W in wrist
  }

  @Override
  public void periodic() {

    switch(wristStates) { // TODO: figure out target angles

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        turnPID.setSetpoint(Constants.Wrist.targetAngle); // make a constant for the target
        motor.set(turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint()));
        break;

      case HANDOFF:
        turnPID.setSetpoint(Constants.Wrist.targetAngle); // make a constant for the target
        motor.set(turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint()));
        break;

      case SPEAKER:
        turnPID.setSetpoint(Constants.Wrist.targetAngle);
        motor.set(turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint()));
        break;

      case SPEAKER_HIGH:
        turnPID.setSetpoint(Constants.Wrist.targetAngle);
        motor.set(turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint()));
        break;
        
      case AMP:
        turnPID.setSetpoint(Constants.Wrist.targetAngle); // make a constant for the target
        motor.set(turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint()));
        break;

      case MANUAL:
        manual();
        break;
    }
  }

  public void setState(WristStates wristStates) {
    this.wristStates = wristStates;
  }

  public double getAngleofMotor() { // TODO: make sure angles are correct 

    double deg = Conversions.falconToDegrees(motor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public void manual() { // TODO: find deadband + correct speed
    double speed = joyvalue.get();

    turnPID.setSetpoint(turnPID.getSetpoint() + speed);

    double finalspeed = turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint());
    motor.set(finalspeed);
  }
}
