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
    turnPID.enableContinuousInput(-180, 180); // wrist isnt continuous, dont need this :P
  }

  public double getAngleofMotor() { // TODO: make sure angles are correct 
    // Put methods under constructor and periodic for organization

    double deg = Conversions.falconToDegrees(motor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public void setWristState(WristStates wristStates, Supplier<Double> joyvalue) { // just put this in periodic, dont need function for it 
    this.wristStates = wristStates;
    // Remove wriststate instance var, make function setState(state) instead, have switch case check state instance var
    // make joystickSupplier instance var

    switch(wristStates) { // TODO: figure out target angles

      case NONE:
        // none = motor.set(0) 
        targetAngle = 0; // remove
        turnPID.setSetpoint(targetAngle); // remove
        break;

      case STOWED:
        targetAngle = 0; // remove, set pid directly
        turnPID.setSetpoint(targetAngle); // make a constant for the target
        break;

      case HANDOFF:
        targetAngle = 0; // remove, set pid directly
        turnPID.setSetpoint(targetAngle); // make a constant for the target
        break;

      case SPEAKER:
        targetAngle = 0; // use conversion file to get angle, dont use preset since the shooting angle changes depending on current pos
        turnPID.setSetpoint(targetAngle);
        break;

      case SPEAKER_HIGH:
        targetAngle = 0; // use conversion file to get angle, dont use preset since the shooting angle changes depending on current pos
        turnPID.setSetpoint(targetAngle);
        break;
        
      case AMP:
        targetAngle = 0; // remove, set pid directly
        turnPID.setSetpoint(targetAngle); // make a constant for the target
        break;

      case MANUAL:
        manual(joyvalue);
        break;

    }

    motor.set(turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint())); // move this under each case that needs it, since some cases dont run motor at all
  }

  public void manual(Supplier<Double> joyvalue) { // TODO: find deadband + correct speed
    // use joystickSupplier instance var, dont need param
    double speed = joyvalue.get();
    double finalspeed; // dont need to define here, see line 110

    if (speed < -0.03) { // Dont need if statement, simply one line of turnPID.setSetpoint(turnPID.getSetpoint() + speed);
      turnPID.setSetpoint(turnPID.getSetpoint() + 0.01);
    }
    else if (speed > 0.03) {
      turnPID.setSetpoint(turnPID.getSetpoint() + 0.01);
    }
    else {
      turnPID.setSetpoint(turnPID.getSetpoint() + 0.01); 
    }
    finalspeed = turnPID.calculate(getAngleofMotor(), turnPID.getSetpoint()); // define here :)
    motor.set(finalspeed);
  }

  @Override
  public void periodic() {}
}
