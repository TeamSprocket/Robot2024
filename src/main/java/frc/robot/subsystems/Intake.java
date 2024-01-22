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

    private IntakeState intakeState;

    double idleSpeed = 0;
    double activeSpeed = 0;

    public enum IntakeState {
        NONE,
        STOWED,
        INTAKE,
        WAIT_HANDOFF,
        HANDOFF
    }


    public Intake() {
        intakeState = IntakeState.NONE;

        rollIntake.setInverted(false);
        pivotIntake.setInverted(false);

        rollIntake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
        rollIntake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));
        
        pivotIntake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
        pivotIntake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));

        ShuffleboardPIDTuner.addSlider("kIdleSpeed", 0, 1, 0.1);

        //intake.setNeutralMode(NeutralMode.Brake);
    }

    public void moveClaw(double output) {
        if (output > 0) {
            idleSpeed = ShuffleboardPIDTuner.get("kIdleSpeed");
        } else if (output < 0) {
            idleSpeed = -ShuffleboardPIDTuner.get("kIdleSpeed");
        }

        activeSpeed = idleSpeed + output;
    }

    public void clearStickyFaults() {
        rollIntake.clearStickyFaults();
        pivotIntake.clearStickyFaults();
    }

    public double getPivotVelocity() {
        return pivotIntake.getSelectedSensorVelocity();
    }
    public double getRollVelocity() {
        return rollIntake.getSelectedSensorVelocity();
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

    public void setPivotAngle(double currentAngle, double setpoint){
        double output = pivotPID.calculate(currentAngle, setpoint);
        pivotIntake.set(output); 
    }

    @Override
    public void periodic() {

        switch (intakeState) {
            case NONE:
            pivotIntake.set(0);

                break;
            case STOWED:
                setPivotAngle(getPivotAngle(), 0);//stowed setpoint
                

                break;
            case INTAKE:
                setPivotAngle(getPivotAngle(), 0);//intake setpoint
                //rollIntake.set(ControlMode.PercentOutput, activeSpeed);
                hasGamePiece();
                // SmartDashboard.putNumber("[Claw] RPM", getVelocity());

                break;
            case WAIT_HANDOFF:
                setPivotAngle(getPivotAngle(), 0); //handoff setpoint
                break;
            case HANDOFF:

                break;
        }
    }
    
    public void setState(IntakeState state) {
        intakeState = state;
    }

    public void hasGamePiece() {
        if (rollIntake.getStatorCurrent() > Constants.Intake.kCurrentThreshold) { 
            rollIntake.set(0);
        } else {
            rollIntake.set(ControlMode.PercentOutput, activeSpeed);
        }
}
}