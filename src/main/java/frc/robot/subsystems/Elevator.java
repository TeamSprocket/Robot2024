package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  Supplier <Double> joyvalue;
  double output;
  

  public static enum ElevatorStates {
    NONE,
    STOWED,
    HANDOFF,
    SPEAKER,
    SPEAKER_HIGH,
    AMP,
    MANUAL
  }

  private ElevatorStates state = ElevatorStates.NONE;
  private ElevatorStates lastState = ElevatorStates.NONE;


  private WPI_TalonFX motorLeft = new WPI_TalonFX(RobotMap.Elevator.Left);
  private WPI_TalonFX motorRight = new WPI_TalonFX(RobotMap.Elevator.Right);

  PIDController pidControllerLeft = new PIDController(Constants.Elevator.kPElevator, Constants.Elevator.kIElevator, Constants.Elevator.kDLeft);
  PIDController pidControllerRight = new PIDController(Constants.Elevator.kPElevator, Constants.Elevator.kIElevator, Constants.Elevator.kDRight);
  
  public Elevator(Supplier<Double> joyvalue) {
    motorRight.follow(motorLeft);
    this.joyvalue = joyvalue;
  }



  @Override
  public void periodic() {
    switch (state) {
        
      case NONE:
        motorLeft.set(0);
        break;
        
      case STOWED:
        pidControllerLeft.setSetpoint(Constants.Elevator.kStowedHeight);
        motorLeft.set(pidControllerLeft.calculate(motorLeft.getSelectedSensorPosition()));
        break;

      case HANDOFF:
        pidControllerLeft.setSetpoint(Constants.Elevator.kHandoffHeight);
        motorLeft.set(pidControllerLeft.calculate(motorLeft.getSelectedSensorPosition()));
        break;

      case SPEAKER:
        pidControllerLeft.setSetpoint(Constants.Elevator.kSpeakerHeight);
        motorLeft.set(pidControllerLeft.calculate(motorLeft.getSelectedSensorPosition()));
        break;  
        
      case SPEAKER_HIGH:
        pidControllerLeft.setSetpoint(Constants.Elevator.kSpeakerHighHeight);
        motorLeft.set(pidControllerLeft.calculate(motorLeft.getSelectedSensorPosition()));
        break;
        
      case AMP:
        pidControllerLeft.setSetpoint(Constants.Elevator.kHeight);
        motorLeft.set(pidControllerLeft.calculate(motorLeft.getSelectedSensorPosition()));
        
      case MANUAL:
        manual();
        break;
    }
   
  }
  public void manual() { // TODO: find deadband + correct speed
    double speed = joyvalue.get();
    pidControllerLeft.setSetpoint(motorLeft.getSelectedSensorPosition() + speed);
    motorLeft.set(pidControllerLeft.calculate(pidControllerLeft.getSetpoint()));
  }
}