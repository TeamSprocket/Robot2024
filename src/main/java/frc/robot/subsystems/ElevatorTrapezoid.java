package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.util.function.Supplier;

import org.apache.commons.lang3.Conversion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
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


  private WPI_TalonFX motorLeft = new WPI_TalonFX(RobotMap.Elevator.Left);
  private WPI_TalonFX motorRight = new WPI_TalonFX(RobotMap.Elevator.Right);  
    PIDController pidControllerLeft = new PIDController(Constants.Elevator.kPElevator, Constants.Elevator.kIElevator, Constants.Elevator.kDElevator);
    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));

  public ElevatorTrapezoid() {
    motorRight.follow(motorLeft);
  }

  @Override
  public void periodic() {
    State trapState = profile.calculate(5, new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));
    pidControllerLeft.calculate(motorLeft.getSelectedSensorPosition(),trapState.position);
  }

  
}