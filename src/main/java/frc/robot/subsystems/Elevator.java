package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private WPI_TalonFX motorLeft = new WPI_TalonFX(0);
  private WPI_TalonFX motorRight = new WPI_TalonFX(0);

  WPI_TalonFX motorLeft = new WPI_TalonFX(RobotMap.Elevator.Left);
  WPI_TalonFX motorRight = new WPI_TalonFX(RobotMap.ELEVATOR.Right);

  PIDController motorLeft = new PIDController(Constants.Elevator.kPElevator, Constants.Shooter.kIElevator, Constants.Elevator.kDLeft);
  PIDController motorRight = new PIDController(Constants.Elevator.kPIndexer, Constants.Shooter.kIElevator, Constants.Elevator.kDRight);
  
  public Elevator() {
    motorRight.follow(motorLeft);
  }

  @Override
  public void periodic() {
    swtich (state) {
      case NONE:
        motorLeft.set(0)
        
    // This method will be called once per scheduler run
  }
  public void goingup(){
    motorLeft.set(0.5);
  }
  public void goingdown(){
    motorLeft.set(-0.5);
  }
  public void stop(){
    motorLeft.set(0);

  }

}



