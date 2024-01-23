// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private final WPI_TalonFX rollIntake = new WPI_TalonFX(RobotMap.Intake.ROLL_INTAKE);

    private final WPI_TalonFX pivotIntake = new WPI_TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    PIDController pivotPID = new PIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot);

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

        rollIntake.setNeutralMode(NeutralMode.Coast);
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
        double output = pivotPID.calculate(getPivotAngle(), setpoint);
        pivotIntake.set(output); 
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
       return rollIntake.getStatorCurrent() > Constants.Intake.kCurrentThreshold;
    }


}