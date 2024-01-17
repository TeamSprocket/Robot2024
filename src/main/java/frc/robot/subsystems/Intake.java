// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private final WPI_TalonFX claw = new WPI_TalonFX(RobotMap.Claw.CLAW);
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
        claw.setInverted(false);
        claw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 1.0));
        claw.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 1.0));

        ShuffleboardPIDTuner.addSlider("kIdleSpeed", 0, 1, 0.1);

        claw.setNeutralMode(NeutralMode.Brake);
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
        return claw.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {

        switch (intakeState) {
            case NONE:

                break;
            case STOWED:

                break;
            case INTAKE:
                claw.set(ControlMode.PercentOutput, activeSpeed);
                SmartDashboard.putNumber("[Claw] RPM", getVelocity());

                break;
            case WAIT_HANDOFF:

                break;
            case HANDOFF:

                break;
        }
    }
}