package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Elevator extends SubsystemBase {
  Supplier <Double> joyvalue;
  double output;
  

  public static enum ElevatorStates {
    NONE,
    STOWED,
    AMP,
    MANUAL
  }
  private ElevatorStates state = ElevatorStates.STOWED;

  private TalonFX elevatorMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_LEFT);
  private TalonFX elevatorFollowerMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_RIGHT);

  // ProfiledPIDController pidController = new ProfiledPIDController(Constants.Elevator.kPIDElevator.kP, Constants.Elevator.kPIDElevator.kI, Constants.Elevator.kPIDElevator.kD,
  //   new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAccel));
  PIDController pidController = new PIDController(Constants.Elevator.kPIDElevator.kP, Constants.Elevator.kPIDElevator.kI, Constants.Elevator.kPIDElevator.kD);

  SendableChooser<ElevatorStates> stateChooser = new SendableChooser<ElevatorStates>();

  
  public Elevator() { 
    configMotors();
    
    elevatorMotor.setPosition(0);
    elevatorFollowerMotor.setPosition(0);

    elevatorFollowerMotor.setControl(new StrictFollower(elevatorMotor.getDeviceID()));

    elevatorMotor.setInverted(Constants.Elevator.kLeftMotorIsInverted);
    elevatorFollowerMotor.setInverted(Constants.Elevator.kRightMotorIsInverted);

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);

    ShuffleboardPIDTuner.addSlider("Elevator kP [EL]", 0, 1, Constants.Elevator.kPIDElevator.kP);
    ShuffleboardPIDTuner.addSlider("Elevator kD [EL]", 0, 1, Constants.Elevator.kPIDElevator.kD);
    ShuffleboardPIDTuner.addSlider("Elevator kFF [EL]", 0, 0.1, Constants.Elevator.kPIDElevator.kFF);


    stateChooser.setDefaultOption("NONE", ElevatorStates.NONE);
    stateChooser.addOption("SCORE_AMP", ElevatorStates.STOWED);
    stateChooser.addOption("AMP", ElevatorStates.AMP);
    stateChooser.addOption("Manual", ElevatorStates.MANUAL);

    SmartDashboard.putData("Elevator State Chooser [EL]", stateChooser);
  }



  @Override
  public void periodic() {
    // setState(stateChooser.getSelected());
    
    // pidController.setP(ShuffleboardPIDTuner.get("Elevator kP [EL]"));
    // pidController.setD(ShuffleboardPIDTuner.get("Elevator kP [EL]"));
    // Constants.Elevator.kPIDElevator.kFF = ShuffleboardPIDTuner.get("Elevator kFF [EL]");
    SmartDashboard.putNumber("Target height elevator [EL]", pidController.getSetpoint());

    switch (state) {
        
      case NONE:
        elevatorMotor.set(0);
        break;
        
      case STOWED:
        moveToHeight(Constants.Elevator.kElevatorHeightStowed);
        break;
        
      case AMP:
        moveToHeight(Constants.Elevator.kElevatorHeightAmp);
        break; 

      case MANUAL:
        manual();
        break;
    }


    SmartDashboard.putNumber("Elevator Height M [EL]", getHeight());
   
  }



  public void setState(ElevatorStates state) {
    this.state = state;
  }


  public void moveToHeight(double targetHeight) {
    pidController.setSetpoint(targetHeight);
    double currentHeight = getHeight();

    double motorOutput = 0.0;

    if (Math.abs(currentHeight - targetHeight) > Constants.Elevator.kFFtoPIDTransitionToleranceM) {
      motorOutput = Constants.Elevator.kElevatorVelocity * Util.getSign(targetHeight - currentHeight);
    } else {
      motorOutput = pidController.calculate(currentHeight) + Constants.Elevator.kPIDElevator.kFF;
    }

    motorOutput = Util.minmax(motorOutput, -1.0 * Constants.Elevator.kElevatorMotorMaxOutput, Constants.Elevator.kElevatorMotorMaxOutput);
    elevatorMotor.set(motorOutput);
    SmartDashboard.putNumber("Elevator PID Output [EL]", motorOutput);
  }


  // public double getScaledFFWithHeight() {
  //   return Constants.Elevator.kPIDElevator.kFF * Constants.Elevator.kFFScaleWithHeight + Constants.Elevator.kFFBaseWithHeight;
  // }


  public double getHeight() {
    return Conversions.falconToMeters(elevatorMotor.getPosition().getValueAsDouble(), Constants.Elevator.kElevatorWinchCircumM, Constants.Elevator.kElevatorGearRatio);
  }


  public void manual() { // TODO: find deadband + correct speed
    // double speed = joyvalue.get();
    // pidControllerLeft.setSetpoint(motorLeft.getSelectedSensorPosition() + speed);
    // motorLeft.set(pidControllerLeft.calculate(pidControllerLeft.getSetpoint()));

    elevatorMotor.set(0.0);
  }

  private void configMotors() {
    // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    // currentLimitsConfigs.withSupplyCurrentLimit(Constants.Elevator.kSupplyCurrentLimit);
    // currentLimitsConfigs.withSupplyCurrentLimitEnable(true);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // motorConfig.withCurrentLimits(currentLimitsConfigs);

    elevatorMotor.getConfigurator().apply(motorConfig);
    elevatorFollowerMotor.getConfigurator().apply(motorConfig);
  }
}


