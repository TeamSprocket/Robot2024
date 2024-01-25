// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

//phoenix imports for pivot intake
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;


//spark max imports for roll intake
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private final CANSparkMax rollIntake = new CANSparkMax(RobotMap.Intake.ROLL_INTAKE, MotorType.kBrushless);

    private final WPI_TalonFX pivotIntake = new WPI_TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    private final TrapezoidProfile.Constraints pivotProfileConstraints = new TrapezoidProfile.Constraints(Constants.Intake.kPivotMaxVelocity, Constants.Intake.kPivotMaxAccel);
    private final ProfiledPIDController pivotPIDProfiled = new ProfiledPIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot, pivotProfileConstraints, 0.02);
    private final SimpleMotorFeedforward pivotFeedForward = new SimpleMotorFeedforward(0,0);

    //private final TrapezoidProfile pivotProfile = new TrapezoidProfile(pivotProfileConstraints);
    //private final TrapezoidProfile.State goal = new TrapezoidProfile.State();
    //private final TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


    TalonFXConfiguration config = new TalonFXConfiguration();

    private IntakeStates state = IntakeStates.NONE;

    double idleSpeed = 0;
    double activeSpeed = 0;

    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKE,
        WAIT_HANDOFF,
        HANDOFF
    }


    public Intake() {
        rollIntake.setInverted(false);
        pivotIntake.setInverted(false);

        rollIntake.setIdleMode(IdleMode.kCoast);
        pivotIntake.setNeutralMode(NeutralMode.Brake);
    }

   
    public double getPivotPosition() {
        return pivotIntake.getSelectedSensorPosition();
    }

    public double getPivotAngle() {
        double deg = Conversions.falconToDegrees(getPivotPosition(), Constants.Intake.kPivotIntakeGearRatio);
        deg %= 360;
        deg = (deg < 0) ? deg + 360 : deg; 
        return deg;
    }

    public void runPivotToSetpoint(double setpoint){
        /*Don't know which calculate methods to use */
        //setpoint = pivotProfile.calculate(0.02, setpoint, goal);
        double output = pivotPIDProfiled.calculate(getPivotAngle(), setpoint); 
        pivotIntake.setVoltage(output + pivotFeedForward.calculate(pivotPIDProfiled.getSetpoint().velocity, 0)); 
    }

    @Override
    public void periodic() {

        switch (state) {
            case NONE:
                pivotIntake.set(0);
                rollIntake.set(0);
                break;


            case STOWED:
                runPivotToSetpoint(Constants.Intake.kPivotAngleStowed);
                rollIntake.set(Constants.Intake.kRollSpeedStowed);
                break;


            case INTAKE:
                runPivotToSetpoint(Constants.Intake.kPivotAngleIntake);
                rollIntake.set(Constants.Intake.kRollSpeedIntake);
                break;

            case WAIT_HANDOFF:
                runPivotToSetpoint(Constants.Intake.kPivotAngleWaitHandoff); 
                rollIntake.set(Constants.Intake.kRollSpeedWaitHandoff);
                break;


            case HANDOFF:
                runPivotToSetpoint(Constants.Intake.kPivotAngleHandoff); 
                rollIntake.set(Constants.Intake.kRollSpeedHandoff);
                break;
        }
    }
    
    public void setState(IntakeStates state) {
        this.state = state;
    }

    public boolean hasDetectedNote() {
       return rollIntake.getOutputCurrent() > Constants.Intake.kCurrentThreshold;
    }


}