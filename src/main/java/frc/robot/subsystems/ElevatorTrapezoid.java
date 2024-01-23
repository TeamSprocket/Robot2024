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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class ElevatorTrapezoid extends TrapezoidProfileSubsystem {


  private WPI_TalonFX motorLeft = new WPI_TalonFX(RobotMap.Elevator.Left);
  private WPI_TalonFX motorRight = new WPI_TalonFX(RobotMap.Elevator.Right);  
  
  public void follow() {
    motorRight.follow(motorLeft);
  }
  

private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          0.01, 0.01,
          0.01, 0.01);

  PIDController pidControllerLeft = new PIDController(Constants.Elevator.kPElevator, Constants.Elevator.kIElevator, Constants.Elevator.kDElevator);
  

  public ElevatorTrapezoid() {super(
new TrapezoidProfile.Constraints(
            1, 0.5),
        0.2);
  
  }


  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    motorLeft.setSetpoint(
        
        (motorLeft.getSelectedSensorPosition(), setpoint.position, feedforward / 12.0));
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }




  
}