// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
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

  PIDController pidController = new PIDController(Constants.ShooterPivot.kPID.kP, Constants.ShooterPivot.kPID.kI, Constants.ShooterPivot.kPID.kD);

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
    SPEAKER_SUBWOOFER,
    CROSSFIELD  
  }

  ShooterPivotStates state = ShooterPivotStates.NONE;
  ShooterPivotStates lastState = ShooterPivotStates.NONE;

  /**
   * Constructor for shooterPivot
   * @param joystickSupplier
   * @param botPoseSupplier
   */
  public ShooterPivot(Supplier<Double> joystickSupplier, Supplier<Translation3d> botPoseSupplier) {

    configMotors();

    this.joystickSupplier = joystickSupplier;
    this.botPoseSupplier = botPoseSupplier;

    //configures the motors
    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30)); // test values later
    
    pidController.setTolerance(Constants.ShooterPivot.kAtGoalTolerance);

    //Adds options for the selection thing
    selectShooterPivotState.setDefaultOption("NONE", ShooterPivotStates.NONE);
    selectShooterPivotState.addOption("STOWED", ShooterPivotStates.STOWED);
    selectShooterPivotState.addOption("INTAKE", ShooterPivotStates.INTAKE);
    selectShooterPivotState.addOption("SPEAKER PODIUM", ShooterPivotStates.SPEAKER_PODIUM);
    SmartDashboard.putData("State Selector [SP]", selectShooterPivotState);

    ShuffleboardIO.addSlider("shooterPivot kP", 0.0, 0.01, Constants.ShooterPivot.kPID.kP);
    ShuffleboardIO.addSlider("shooterPivot kD", 0.0, 0.001, 0.0);
    ShuffleboardIO.addSlider("kHorizontalAngle ShooterPivot [SP]", 5.0, 90.0, Constants.ShooterPivot.kTargetAngleStowed);
  }

  @Override
  public void periodic() {
    debug();

    //switches the state
    switch(state) {

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleStowed);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case EJECT_NOTE:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleEject);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;


      case INTAKE:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleIntake);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case INDEXING:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleIndexing);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case SPEAKER_SUBWOOFER:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
      break;

      case SPEAKER_PODIUM:

        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAnglePodium);
        motor.set(motorspeed);

        break;
 
      case CROSSFIELD:

        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleCrossfield); 

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);

        break;
    }

    lastState = state;
    SmartDashboard.putNumber("ShooterPivot Angle [SP]", getShooterPivotAngle());
    SmartDashboard.putBoolean("Shooter Pivot atGoal [SP]", atGoal());
  }

  /**
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

  /**
   * Gets the pivot speed
   * @param targetAngle Uses the targetAngle to get a set pivot speed.
   * @return
   */
  public double getPivotSpeed(double targetAngle) {
    double pivotSpeed;
    pidController.setSetpoint(targetAngle);
    double currentAngle = getShooterPivotAngle();
    double PIDoutput = pidController.calculate(currentAngle);
    double signedKFF = Constants.ShooterPivot.kPID.kFF * Util.getSign(PIDoutput);

    if(Math.abs(targetAngle - currentAngle) > Constants.ShooterPivot.kFFtoPIDTransitionTolerance) {
      pivotSpeed = Constants.ShooterPivot.kFFPivot * Util.getSign(PIDoutput);
    } else {
      pivotSpeed = PIDoutput + signedKFF;
      if (pidController.atSetpoint()) {
        pivotSpeed = 0;
      }
    }

    pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
    return pivotSpeed;
  }

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
    SmartDashboard.putNumber("Angle in Degrees", getShooterPivotAngle());
    SmartDashboard.putNumber("Shot Target Angle [SP]", Util.getTargetShotAngleDeg(botPoseSupplier.get(), Util.getSpeakerTargetBasedOnAllianceColor()));
    SmartDashboard.putString("State [SP]", state.toString());
  }

  /**
   * 
   * @return If the shooterPivot is at the setpoint.
   */
  public boolean atGoal() {
    return pidController.atSetpoint();
  }

  public void zeroPosition() {
    motor.setPosition(0);
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  //Configurate the motors.
  private void configMotors() {
    
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig);
  }
}
