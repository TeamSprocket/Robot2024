// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//phoenix imports for pivot intake
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;


//spark max imports for roll intake
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.Util;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private final CANSparkMax rollIntake = new CANSparkMax(RobotMap.Intake.ROLL_INTAKE, MotorType.kBrushless);
    private final TalonFX pivotIntake = new TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    ProfiledPIDController profiledPIDController;

    private IntakeStates state = IntakeStates.NONE;
    private IntakeStates lastState = IntakeStates.NONE;
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKE,
        WAIT_HANDOFF,
        HANDOFF
    }


    public Intake() {
        TrapezoidProfile.Constraints pivotProfileConstraints = new TrapezoidProfile.Constraints(Constants.Intake.kPivotMaxVelocity, Constants.Intake.kPivotMaxAccel);
        profiledPIDController = new ProfiledPIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot, pivotProfileConstraints);

        rollIntake.setInverted(Constants.Intake.kIsRollInverted);
        pivotIntake.setInverted(Constants.Intake.kIsPivotInverted);

        rollIntake.setIdleMode(IdleMode.kBrake);
        pivotIntake.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void periodic() {

        switch (state) {
            case NONE:
                pivotIntake.set(0);
                rollIntake.set(0);
                break;

            case STOWED:
                // if (lastState != IntakeStates.STOWED) {
                    profiledPIDController.setGoal(Constants.Intake.kPivotAngleStowed);
                // }
                pivotIntake.set(profiledPIDController.calculate(getPivotAngle()));
                rollIntake.set(Constants.Intake.kRollSpeedStowed);
                break;

            case INTAKE:
                // if (lastState != IntakeStates.INTAKE) {
                    profiledPIDController.setGoal(Constants.Intake.kPivotAngleIntake);
                // }
                pivotIntake.set(profiledPIDController.calculate(getPivotAngle()));
                rollIntake.set(Constants.Intake.kRollSpeedIntake);
                break;

            // case WAIT_HANDOFF:
            //     // if (lastState != IntakeStates.WAIT_HANDOFF) {
            //         profiledPIDController.setGoal(Constants.Intake.kPivotAngleWaitHandoff);
            //     // }
            //     pivotIntake.set(profiledPIDController.calculate(getPivotAngle()));
            //     rollIntake.set(Constants.Intake.kRollSpeedWaitHandoff);
            //     break;


            // case HANDOFF:
            //     // if (lastState != IntakeStates.HANDOFF) {
            //         profiledPIDController.setGoal(Constants.Intake.kPivotAngleHandoff);
            //     // }
            //     pivotIntake.set(profiledPIDController.calculate(getPivotAngle()));
            //     rollIntake.set(Constants.Intake.kRollSpeedHandoff);
            //     break;
        }

        clearStickyFaults();
        lastState = state;
    }
    
    public void setState(IntakeStates state) {
        this.state = state;
    }

    public IntakeStates getState() {
        return state;
    }


    public double getPivotAngle() {
        double deg = Conversions.falconToDegrees(pivotIntake.getPosition().getValueAsDouble(), Constants.Intake.kPivotIntakeGearRatio);
        deg %= 360;
        deg = (deg < 0) ? deg + 360 : deg; 
        return deg;
    }

    // public void runPivotToSetpoint(double setpoint){
    //     /*Don't know which calculate methods to use */
    //     //setpoint = pivotProfile.calculate(0.02, setpoint, goal);
    //     double output = pivotPIDProfiled.calculate(getPivotAngle(), setpoint); 
    //     pivotIntake.setVoltage(output + pivotFeedForward.calculate(pivotPIDProfiled.getSetpoint().velocity, 0)); 
    // }
    

    public boolean hasDetectedNote() {
       return rollIntake.getOutputCurrent() > Constants.Intake.kHasNoteCurrentThreshold;
    }

  public boolean atGoal() {
    double goal = profiledPIDController.getGoal().position;
    return Util.inRange(getPivotAngle(), (goal - Constants.Intake.kAtGoalTolerance), (goal + Constants.Intake.kAtGoalTolerance));
  }

  public void clearStickyFaults() {
    pivotIntake.clearStickyFaults();
  }
}