package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.util.function.Supplier;

import org.apache.commons.lang3.Conversion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.util.Conversions;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;


//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
public class ElevatorTrapezoid extends SubsystemBase {

  public static enum ElevatorStates {
    NONE,
    STOWED,
    HANDOFF,
    SPEAKER,
    SPEAKER_HIGH,
    AMP,
    TRAP,
    MANUAL
  }
  private double goal;
  private ElevatorStates state = ElevatorStates.NONE;

  private WPI_TalonFX motorLeft = new WPI_TalonFX(RobotMap.Elevator.Left);
  private WPI_TalonFX motorRight = new WPI_TalonFX(RobotMap.Elevator.Right);  
    // TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityRadPerSecond, Constants.Wrist.kMaxAccelerationRadPerSecSquared));

    ProfiledPIDController elevatorPID = new ProfiledPIDController(Constants.Elevator.kPElevator, Constants.Elevator.kIElevator, Constants.Elevator.kPElevator,new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity,Constants.Elevator.kMaxAcceleration));

  public ElevatorTrapezoid() {
    motorRight.follow(motorLeft);
  }
  public void setState(ElevatorStates state) {
    this.state = state;
  }
  public void setGoal(double goal){
    this.goal = goal;

  } 

  @Override
  public void periodic() {

    switch (state) {
      case MANUAL:
        elevatorPID.setGoal(goal);
        motorLeft.set(elevatorPID.calculate(motorLeft.getSelectedSensorPosition()));
        break;
    
      default:
        break;
    }
  }

  
}