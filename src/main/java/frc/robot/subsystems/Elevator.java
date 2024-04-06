package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
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

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.Timer;

public class Elevator extends SubsystemBase {
  Supplier <Double> joyvalue;
  double output;
  Timer timer = new Timer();
  

  public static enum ElevatorStates {
    NONE,
    STOWED,
    AMP,
    MANUAL
  }
  private ElevatorStates state = ElevatorStates.NONE;

  private TalonFX elevatorMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_LEFT);
  private TalonFX elevatorFollowerMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_RIGHT);
  private MotionMagicVoltage mmV = new MotionMagicVoltage(0);

  PIDController pidController = new PIDController(Constants.Elevator.kPIDElevator.kP, Constants.Elevator.kPIDElevator.kI, Constants.Elevator.kPIDElevator.kD);

  SendableChooser<ElevatorStates> stateChooser = new SendableChooser<ElevatorStates>();
  TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

  private final MotionMagicExpoTorqueCurrentFOC PoseRequest =
        new MotionMagicExpoTorqueCurrentFOC(-0.12)
            .withSlot(0);
  
  public Elevator() {
    elevatorFollowerMotor.setControl(new StrictFollower(elevatorMotor.getDeviceID()));

    elevatorMotor.setInverted(Constants.Elevator.kLeftMotorIsInverted);
    elevatorFollowerMotor.setInverted(Constants.Elevator.kRightMotorIsInverted);

    talonFXConfigs.withMotionMagic(
      new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(Constants.Elevator.kMotionMagicCruiseVelocity)
        .withMotionMagicAcceleration(Constants.Elevator.kMotionMagicAcceleration)
        .withMotionMagicJerk(Constants.Elevator.kMotionMagicJerk)
    );

    talonFXConfigs.withSlot0(
      new Slot0Configs()
        .withKP(Constants.Elevator.kPIDElevator.getP())
        .withKI(Constants.Elevator.kPIDElevator.getI())
        .withKD(Constants.Elevator.kPIDElevator.getD())
    );

    talonFXConfigs.withFeedback(
      new FeedbackConfigs()
        .withSensorToMechanismRatio(117.6)
    );

    talonFXConfigs.withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
    );
    
    elevatorMotor.getConfigurator().apply(talonFXConfigs);
    elevatorFollowerMotor.getConfigurator().apply(talonFXConfigs);

    ShuffleboardPIDTuner.addSlider("Elevator kP [EL]", 0, 1, Constants.Elevator.kPIDElevator.kP);
    ShuffleboardPIDTuner.addSlider("Elevator kD [EL]", 0, 1, Constants.Elevator.kPIDElevator.kD);
    ShuffleboardPIDTuner.addSlider("Elevator kFF [EL]", 0, 0.1, Constants.Elevator.kPIDElevator.kFF);


    stateChooser.setDefaultOption("NONE", ElevatorStates.NONE);
    stateChooser.addOption("SCORE_AMP", ElevatorStates.STOWED);
    stateChooser.addOption("AMP", ElevatorStates.AMP);
    stateChooser.addOption("Manual", ElevatorStates.MANUAL);

    SmartDashboard.putData("Elevator State Chooser [EL]", stateChooser);
    timer.start();
  }



  @Override
  public void periodic() {
    //setState(stateChooser.getSelected());
    SmartDashboard.putNumber("Elevator Position[EL]", getPosition());
    SmartDashboard.putNumber("Timer for Elevator [EL]", timer.get());
    SmartDashboard.putNumber("Elevator Height[EL]", getHeight());
    pidController.setP(ShuffleboardPIDTuner.get("Elevator kP [EL]"));
    pidController.setD(ShuffleboardPIDTuner.get("Elevator kD [EL]"));
    
    //TESTER
    // if (timer.get() > 5.0) {
    //   if (state == ElevatorStates.AMP) {
    //     setState(ElevatorStates.STOWED);
    //   } else {
    //     setState(ElevatorStates.AMP);
    //   }
    //   timer.stop();
    //   timer.reset();
    //   timer.start();
    // }

    switch (state) {
        
      case NONE:
        elevatorMotor.set(0);
        break;
        
      case STOWED:
        elevatorMotor.setPosition(5);
        break;
        
      case AMP:
        elevatorMotor.setPosition(10);
        break; 

      case MANUAL:
        manual();
        break;
    }

  }



  public void setState(ElevatorStates state) {
    this.state = state;
  }

  public void moveToHeight(double targetHeight) {
    // double motorOutput = pidController.calculate(getHeight(), targetHeight);
    // //motorOutput += Constants.Elevator.PIDTuner.kFF;
    // motorOutput += ShuffleboardPIDTuner.get("Elevator kFF [EL]");
    // motorOutput = Util.minmax(motorOutput, -1.0 * Constants.Elevator.kElevatorMotorMaxOutput, Constants.Elevator.kElevatorMotorMaxOutput);
    // elevatorMotor.set(motorOutput);
  }

  public double getHeight() {
    return Conversions.falconToMeters(elevatorMotor.getPosition().getValueAsDouble(), Constants.Elevator.kElevatorWinchCircumM, Constants.Elevator.kElevatorGearRatio);
  }

  public double getPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }


  public void manual() { // TODO: find deadband + correct speed
    // double speed = joyvalue.get();
    // pidControllerLeft.setSetpoint(motorLeft.getSelectedSensorPosition() + speed);
    // motorLeft.set(pidControllerLeft.calculate(pidControllerLeft.getSetpoint()));

    elevatorMotor.set(0.0);
  }

  
}


