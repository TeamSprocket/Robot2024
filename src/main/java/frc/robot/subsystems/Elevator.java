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
import frc.util.ShuffleboardIO;
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
  double speed = 0.0;
  boolean hasClimbed = false;
  double topHeight, bottomHeight;

  /**
   * Elevator States: Different states for the elvator to be in to do. They do different things in each state to run to a specific position for a function.
   * Use setState to set to an elevator state, and make sure to import elevatorStates in the command so you can use the state to put it in.
   * */  
  public static enum ElevatorStates {
    NONE,
    STOWED,
    AMP,
    CLIMB_HOLD_TOP,
    CLIMB_HOLD_BOTTOM,
    FIND_TOP, 
    FIND_BOTTOM,
    MANUAL
  }
  private ElevatorStates state = ElevatorStates.STOWED;

  public static enum ClimbStates {
    UP,
    DOWN,
    HOLD,
    NONE
  }
  private ClimbStates climbState = ClimbStates.NONE;


  // Different motors for the elevator.
  private TalonFX elevatorMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_LEFT);
  private TalonFX elevatorFollowerMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_RIGHT);
  private MotionMagicVoltage mmV = new MotionMagicVoltage(0);

  // ProfiledPIDController pidController = new ProfiledPIDController(Constants.Elevator.kPIDElevator.kP, Constants.Elevator.kPIDElevator.kI, Constants.Elevator.kPIDElevator.kD,
  //   new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAccel));
  PIDController pidController = new PIDController(Constants.Elevator.kPIDElevator.kP, Constants.Elevator.kPIDElevator.kI, Constants.Elevator.kPIDElevator.kD);

  // State chooser for the things to change the state manually on shuffleBoard.
  SendableChooser<ElevatorStates> stateChooser = new SendableChooser<ElevatorStates>();
  TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

  //Elevator Constructor
  public Elevator(Supplier<Double> joystickValue) { 
    configMotors();

    joyvalue = joystickValue;
    
    elevatorMotor.setPosition(0);
    elevatorFollowerMotor.setPosition(0);

    elevatorFollowerMotor.setControl(new StrictFollower(elevatorMotor.getDeviceID()));

    elevatorMotor.setInverted(Constants.Elevator.kLeftMotorIsInverted);
    elevatorFollowerMotor.setInverted(Constants.Elevator.kRightMotorIsInverted);

    // ShuffleboardIO.addSlider("Elevator kP [EL]", 0, 1, Constants.Elevator.kPIDElevator.kP);
    // ShuffleboardIO.addSlider("Elevator kD [EL]", 0, 1, Constants.Elevator.kPIDElevator.kD);
    // ShuffleboardIO.addSlider("Elevator kFF [EL]", 0, 0.1, Constants.Elevator.kPIDElevator.kFF);


    stateChooser.setDefaultOption("NONE", ElevatorStates.NONE);
    stateChooser.addOption("SCORE_AMP", ElevatorStates.STOWED);
    stateChooser.addOption("AMP", ElevatorStates.AMP);
    stateChooser.addOption("Manual", ElevatorStates.MANUAL);

    SmartDashboard.putData("Elevator State Chooser [EL]", stateChooser);
  }



  @Override
  public void periodic() {

    // Constants.Elevator.kElevatorHeightClimbUp = ShuffleboardIO.getDouble("kElevatorHeightClimbUp [EL]");
    // Constants.Elevator.kElevatorHeightClimbDown = ShuffleboardIO.getDouble("kElevatorHeightClimbDown [EL]");
    // Constants.Elevator.kElevatorMotorMaxOutputClimb = ShuffleboardIO.getDouble("kElevatorMotorMaxOutputClimb [EL]");

    // Constants.Elevator.kElevatorHeightAmp = ShuffleboardIO.getDouble("kElevatorHeightAmp [EL]");

    // setState(stateChooser.getSelected());
    
    // pidController.setP(ShuffleboardIO.getDouble("Elevator kP [EL]"));
    // pidController.setD(ShuffleboardIO.getDouble("Elevator kP [EL]"));
    // Constants.Elevator.kPIDElevator.kFF = ShuffleboardIO.getDouble("Elevator kFF [EL]");
    // SmartDashboard.putNumber("Target height elevator [EL]", pidController.getSetpoint());

    switch (state) {
        
      case NONE:
        elevatorMotor.set(0);
        break;
        
      case STOWED:
        elevatorMotor.setControl(mmV.withSlot(0).withPosition(5));
        break;
        
      case AMP:
        moveToHeight(Constants.Elevator.kElevatorHeightAmp);
        break; 

      case CLIMB_HOLD_TOP:
        moveToTop();
        break;

      case CLIMB_HOLD_BOTTOM:
        moveToHeight(Constants.Elevator.kElevatorHeightClimbDown, Constants.Elevator.kElevatorMotorMaxOutputClimb);
        break;

      case FIND_TOP:
        elevatorMotor.set(Constants.Elevator.kClimbFindTopSpeed);
        break;

      case FIND_BOTTOM:
        elevatorMotor.set(Constants.Elevator.kClimbFindBottomSpeed);
        break;

      case MANUAL:
        manual();
        break;
    }

    // SmartDashboard Shenanigans
    SmartDashboard.putNumber("Elevator Height M [EL]", getHeight());
    SmartDashboard.putNumber("Elevator Left Stator [EL]", elevatorMotor.getStatorCurrent().getValueAsDouble());
   
  }


  /**
   * Sets the state of the elevator
   * @param state
   */
  public void setState(ElevatorStates state) {
    this.state = state;
  }

  /**
   * Moves to a specific height. Uses a feed forward until it reaches a certain setpoint where it converts to PID movement.
   * @param targetHeight
   */
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
    // SmartDashboard.putNumber("Elevator PID Output [EL]", motorOutput);
  }

  /**
   * Move directly to the max height of elevator.
   */
  public void moveToTop() {
    moveToHeight(topHeight);
  }

  // public void moveToBottom() {
  //   moveToHeight(bottomHeight);
  // }

  /**
   * Sets the top height
   * @param topHeight
   */
  public void setTopHeight(double topHeight) {
    this.topHeight = topHeight;
  }

  // public void setBottomHeight(double bottomHeight) {
  //   this.bottomHeight = bottomHeight;
  // }


  public boolean climbHasHooked() {
    return Math.abs(elevatorMotor.getStatorCurrent().getValueAsDouble()) > Constants.Elevator.kClimbHookedCurrentThreshold;
  }

  public boolean climbHasHitTop() {
    return Math.abs(elevatorMotor.getStatorCurrent().getValueAsDouble()) > Constants.Elevator.kClimbFindTopCurrentThreshold;
  }

  public boolean climbHasHitBottom() {
    return Math.abs(elevatorMotor.getStatorCurrent().getValueAsDouble()) > Constants.Elevator.kClimbFindBottomCurrentThreshold;
  }

  /**
   * Moves to the specific height for the elevator.
   * @param targetHeight
   * @param maxOutput
   */
  public void moveToHeight(double targetHeight, double maxOutput) {
    pidController.setSetpoint(targetHeight);
    double currentHeight = getHeight();

    double motorOutput = 0.0;

    if (Math.abs(currentHeight - targetHeight) > Constants.Elevator.kFFtoPIDTransitionToleranceM) {
      motorOutput = Constants.Elevator.kElevatorVelocity * Util.getSign(targetHeight - currentHeight);
    } else {
      motorOutput = pidController.calculate(currentHeight) + Constants.Elevator.kPIDElevator.kFF;
    }

    motorOutput = Util.minmax(motorOutput, -1.0 * maxOutput, maxOutput);
    elevatorMotor.set(motorOutput);
  }

  /**
   * Sets the climb state.
   * @param climbState
   */
  public void setClimbState(ClimbStates climbState) {
      this.climbState = climbState;
  }

  public ClimbStates getClimbState() {
      return climbState;
  }
  
  // public double getScaledFFWithHeight() {
  //   return Constants.Elevator.kPIDElevator.kFF * Constants.Elevator.kFFScaleWithHeight + Constants.Elevator.kFFBaseWithHeight;
  // }

  /**
   * Returns the height of the elevator.
   * @return
   */
  public double getHeight() {
    return Conversions.falconToMeters(elevatorMotor.getPosition().getValueAsDouble(), Constants.Elevator.kElevatorWinchCircumM, Constants.Elevator.kElevatorGearRatio);
  }

  /**
   * Manual controls for the elevator.
   */
  public void manual() { // TODO: find deadband + correct speed
    // double speed = joyvalue.get();
    // pidControllerLeft.setSetpoint(motorLeft.getSelectedSensorPosition() + speed);
    // motorLeft.set(pidControllerLeft.calculate(pidControllerLeft.getSetpoint()));

    double motorOutput = joyvalue.get();
    motorOutput = Util.signedSquare(motorOutput);

    if (motorOutput < 0 && getHeight() <= 0.01) {
      elevatorMotor.set(0);
    } else {
      elevatorMotor.set(Util.minmax(motorOutput, -0.1, 0.1));
    }
  }

  public boolean isAtTargetGoal() {
    if (Math.abs(pidController.getSetpoint() - getHeight()) < 0.01) {
      return true;
    }
    else {
      return false;
    }
  }

  public void zeroPosition() {
    elevatorMotor.setPosition(0);
    elevatorFollowerMotor.setPosition(0);
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


