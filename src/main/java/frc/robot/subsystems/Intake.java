// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

<<<<<<< Updated upstream

import edu.wpi.first.math.controller.PIDController;
=======
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;

/** Add your docs here. */
public class Intake extends SubsystemBase {

<<<<<<< Updated upstream
    private final WPI_TalonFX claw = new WPI_TalonFX(RobotMap.Claw.CLAW);

    private final WPI_TalonFX pivotIntake = new WPI_TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    PIDController pivotPID = new PIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot);

=======
    private final WPI_TalonFX intake = new WPI_TalonFX(RobotMap.Intake.INTAKE);
>>>>>>> Stashed changes
    double idleSpeed = 0;
    double activeSpeed = 0;

    public enum IntakeState {
        NONE,
        STOWED,
        INTAKE,
        WAIT_HANDOFF,
        HANDOFF
    }

    private TalonFX pivotIntake;
    private TalonFX rollIntake;

    private IntakeState intakeState;

    public Intake() {
        intakeState = IntakeState.NONE;
<<<<<<< Updated upstream

        claw.setInverted(false);
        pivotIntake.setInverted(false);

        claw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
        claw.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));
=======
        intake.setInverted(false);
        intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
        intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));
>>>>>>> Stashed changes

        ShuffleboardPIDTuner.addSlider("kIdleSpeed", 0, 1, 0.1);

        intake.setNeutralMode(NeutralMode.Brake);
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
    }

    public double getVelocity() {
        return intake.getSelectedSensorVelocity();
    }

    public double getPivotPosition() {
        return pivotIntake.getSelectedSensorPosition();
    }

    public double getPivotAngle() {
        return Conversions.falconToDegrees(getPivotPosition(), Constants.Intake.kPivotIntakeGearRatio); 
    }

    public void setPivotAngle(double currentAngle, double setpoint){
        double output = pivotPID.calculate(currentAngle, setpoint);
        pivotIntake.set(output); 
    }

    @Override
    public void periodic() {

        switch (intakeState) {
            case NONE:
<<<<<<< Updated upstream
            pivotIntake.set(0);
=======
                intake.set(ControlMode.PercentOutput, 0);
>>>>>>> Stashed changes

                break;
            case STOWED:
                setPivotAngle(getPivotAngle(), 0);//stowed setpoint
                

                break;
            case INTAKE:
<<<<<<< Updated upstream
                setPivotAngle(getPivotAngle(), 0);//intake setpoint
                claw.set(ControlMode.PercentOutput, activeSpeed);
=======
                intake.set(ControlMode.PercentOutput, activeSpeed);
>>>>>>> Stashed changes
                SmartDashboard.putNumber("[Claw] RPM", getVelocity());

                break;
            case WAIT_HANDOFF:
                setPivotAngle(getPivotAngle(), 0); //handoff setpoint
                break;
            case HANDOFF:

                break;
        }
    }
}