package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {

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
  private WPI_TalonFX motorRight = new WPI_TalonFX(RobotMap.ELEVATOR.Right);

  PIDController pidControllerLeft = new PIDController(Constants.Elevator.kPElevator, Constants.Shooter.kIElevator, Constants.Elevator.kDLeft);
  PIDController pidControllerRight = new PIDController(Constants.Elevator.kPIndexer, Constants.Shooter.kIElevator, Constants.Elevator.kDRight);
  
  public Elevator() {
    motorRight.follow(motorLeft);
  }

  @Override
  public void periodic() {
    switch (state) {
        
      case NONE:
        motorLeft.set(0);
        break;
        
      case STOWED:
        pidControllerLeft.setSetpoint(motorLeft.getSelectedSensorPosition());
        double motorLeftMotorOutput = PIDControllerLeft.calculate(motorLeft.getSelectedSensorPosition(), );
        motorLeft.set(motorLeftMotorOutput);
        pidControllerLeft.setSelectedSensorPosition()
        break;
      case HANDOFF:
        
      case SPEAKER:
        
      case SPEAKER_HIGH:
        
      case AMP:
        
      case MANUAL:
       
        break;
    }
   
  }

  public void goingup() {
    motorLeft.set(0.5);
  }

  public void goingdown() {
    motorLeft.set(-0.5);
  }

  public void stop() {
    motorLeft.set(0);
  }
}
