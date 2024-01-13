// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase{

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
    }

    @Override
    public void periodic() {

        switch (intakeState) {
            case NONE:

                break;
            case STOWED:

                break;
            case INTAKE:

                break;
            case WAIT_HANDOFF:

                break;
            case HANDOFF:

                break;
        }
    }
}
