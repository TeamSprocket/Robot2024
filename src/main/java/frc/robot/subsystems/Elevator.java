package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.util.Conversions;
import frc.util.Util;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Elevator extends SubsystemBase {
  SendableChooser<ElevatorStates> selectElevatorState = new SendableChooser<ElevatorStates>();
  public static enum ElevatorStates {
    NONE,
    STOWED,
    HANDOFF,
    SPEAKER,
    SPEAKER_HIGH,
    AMP,
    TRAP,
    CLIMB
  }

  private ElevatorStates state = ElevatorStates.NONE;

  private final TalonFX motorLeft = new TalonFX(RobotMap.Elevator.ELEVATOR_LEFT);
  private final TalonFX motorRight = new TalonFX(RobotMap.Elevator.ELEVATOR_RIGHT);

  ProfiledPIDController profiledPIDController;

  SendableChooser<IntakeStates> selectIntakeState = new SendableChooser<IntakeStates>();
  
  Supplier<Double> leftBumperSupplier;
  Supplier<Double> rightBumperSupplier;

  MotionMagicVoltage mmV = new MotionMagicVoltage(0);
  VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public Elevator(Supplier<Double> leftBumperSupplier, Supplier<Double> rightBumperSupplier) {
    
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Elevator.kElevatorMMCruiseVelocity)
                .withMotionMagicAcceleration(Constants.Elevator.kElevatorMMCruiseAccel)
        );
    
    elevatorConfig.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.Elevator.kElevatorS) 
                .withKV(Constants.Elevator.kElevatorV)
                .withKA(Constants.Elevator.kElevatorA) 
                .withKG(Constants.Elevator.kElevatorG)
                .withKP(Constants.Elevator.kElevatorP) 
                .withKI(Constants.Elevator.kElevatorI)
                .withKD(Constants.Elevator.kElevatorD)
        );
    
    elevatorConfig.withFeedback(
      new FeedbackConfigs()
          .withSensorToMechanismRatio(Constants.Elevator.kElevatorGearRatio)
    );

    motorLeft.getConfigurator().apply(elevatorConfig);
    motorRight.getConfigurator().apply(elevatorConfig);

    mmV.Slot = 0;
    velocityVoltage.Slot = 0;

    motorLeft.setNeutralMode(NeutralModeValue.Brake);
    motorRight.setNeutralMode(NeutralModeValue.Brake);

    motorLeft.setInverted(Constants.Elevator.kIsInvertedLeft);
    motorRight.setInverted(Constants.Elevator.kIsInvertedRight);

    motorRight.setControl(new StrictFollower(motorLeft.getDeviceID()));

    TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAccel);
    profiledPIDController = new ProfiledPIDController(Constants.Elevator.kPElevator, Constants.Elevator.kIElevator, Constants.Elevator.kDElevator, trapezoidProfileConstraints);

    this.leftBumperSupplier = leftBumperSupplier;
    this.rightBumperSupplier = rightBumperSupplier;

      selectElevatorState.setDefaultOption("NONE", ElevatorStates.NONE);
      selectElevatorState.addOption("STOWED", ElevatorStates.NONE);
      selectElevatorState.addOption("HANDOFF", ElevatorStates.HANDOFF);
      selectElevatorState.addOption("SPEAKER", ElevatorStates.SPEAKER);
      selectElevatorState.addOption("SPEAKER_HIGH", ElevatorStates.SPEAKER_HIGH);
      selectElevatorState.addOption("AMP", ElevatorStates.AMP);
      selectElevatorState.addOption("TRAP", ElevatorStates.TRAP);
      selectElevatorState.addOption("CLIMB", ElevatorStates.CLIMB);
       SmartDashboard.putData(selectElevatorState);
    }



  @Override
  public void periodic() {
    // setState(selectElevatorState.getSelected());
    switch (state) {
        
      case NONE:
        motorLeft.set(0);
        motorRight.set(0);
        break;
        
      case STOWED:
        motorLeft.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightStowed));
        motorRight.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightStowed));
        break;

      case HANDOFF:
        motorLeft.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightHandoff));
        break;

      case SPEAKER:
        motorLeft.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightSpeaker));
        break;  
        
      case SPEAKER_HIGH:
        motorLeft.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightSpeakerHigh));
        break;
        
      case AMP:
        motorLeft.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightAmp));
        break; 

      case TRAP:
        motorLeft.setControl(mmV.withPosition(Constants.Elevator.kElevatorHeightTrap));
        
      case CLIMB:
        manual();
        break;
    }
   
    

    clearStickyFaults();
  }


  public void setState(ElevatorStates state) {
    this.state = state;
  }

  public ElevatorStates getState() {
      return state;
  }

  public void manual() { // TODO: find deadband + correct speed
    double speedL = -1.0 * leftBumperSupplier.get();
    double speedR = rightBumperSupplier.get();
    double speed = speedL + speedR;
    speed *= Constants.Elevator.kManualMultiplier;
    Util.deadband(speed, -0.1, 0.1);

    var elevatorRotorSignal = motorLeft.getRotorPosition();

    profiledPIDController.setGoal(elevatorRotorSignal.getValue() + speed);
    motorLeft.set(profiledPIDController.calculate(getHeight()));
  }

  public double getHeight() {
    var elevatorRotorSignal = motorLeft.getRotorPosition();

    return Conversions.falconToMeters(elevatorRotorSignal.getValue(), Constants.Elevator.kElevatorGearCircumM, Constants.Elevator.kElevatorGearRatio);
  }


  public boolean atGoal() {
    double goal = profiledPIDController.getGoal().position;
    return Util.inRange(getHeight(), (goal - Constants.Elevator.kAtGoalTolerance), (goal + Constants.Elevator.kAtGoalTolerance));
  }

  public void clearStickyFaults() {
    motorLeft.clearStickyFaults();
    motorRight.clearStickyFaults();
  }
  
}


