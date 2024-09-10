package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.util.function.Supplier;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

  private final TalonFX motorLeft = new TalonFX(RobotMap.Elevator.Left);
  private final TalonFX motorRight = new TalonFX(RobotMap.Elevator.Right);

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

    motorLeft.setNeutralMode(NeutralMode.Brake);
    motorRight.setNeutralMode(NeutralMode.Brake);

    motorLeft.setInverted(Constants.Elevator.kIsInvertedLeft);
    motorRight.setInverted(Constants.Elevator.kIsInvertedRight);

    motorRight.follow(motorLeft);

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
        break;
        
      case STOWED:
        profiledPIDController.setGoal(Constants.Elevator.kElevatorHeightStowed);
        motorLeft.set(profiledPIDController.calculate(getHeight()));
        break;

      case HANDOFF:
        profiledPIDController.setGoal(Constants.Elevator.kElevatorHeightHandoff);
        motorLeft.set(profiledPIDController.calculate(getHeight()));
        break;

      case SPEAKER:
        profiledPIDController.setGoal(Constants.Elevator.kElevatorHeightSpeaker);
        motorLeft.set(profiledPIDController.calculate(getHeight()));
        break;  
        
      case SPEAKER_HIGH:
        profiledPIDController.setGoal(Constants.Elevator.kElevatorHeightSpeakerHigh);
        motorLeft.set(profiledPIDController.calculate(getHeight()));
        break;
        
      case AMP:
        profiledPIDController.setGoal(Constants.Elevator.kElevatorHeightAmp);
        motorLeft.set(profiledPIDController.calculate(getHeight()));
        break; 

      case TRAP:
        profiledPIDController.setGoal(Constants.Elevator.kElevatorHeightTrap);
        motorLeft.set(profiledPIDController.calculate(getHeight()));
        
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

    profiledPIDController.setGoal(motorLeft.getSelectedSensorPosition() + speed);
    motorLeft.set(profiledPIDController.calculate(getHeight()));
  }

  public double getHeight() {
    return Conversions.falconToMeters(motorLeft.getSelectedSensorPosition(), Constants.Elevator.kElevatorGearCircumM, Constants.Elevator.kElevatorGearRatio);
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


