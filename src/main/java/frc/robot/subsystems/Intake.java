// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//phoenix imports for pivot intake
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;


//spark max imports for roll intake
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private final TalonFX rollIntake = new TalonFX(RobotMap.Intake.ROLL_INTAKE);
    private final TalonFX pivotIntake = new TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    ProfiledPIDController profiledPIDController;

    private IntakeStates state = IntakeStates.NONE;
    private IntakeStates lastState = IntakeStates.NONE;

    SendableChooser<IntakeStates> selectIntakeState = new SendableChooser<IntakeStates>();

    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKE
    }


    public Intake() {
        TrapezoidProfile.Constraints pivotProfileConstraints = new TrapezoidProfile.Constraints(Constants.Intake.kPivotMaxVelocity, Constants.Intake.kPivotMaxAccel);
        profiledPIDController = new ProfiledPIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot, pivotProfileConstraints);

        rollIntake.setInverted(Constants.Intake.kIsRollInverted);
        pivotIntake.setInverted(Constants.Intake.kIsPivotInverted);

        rollIntake.setNeutralMode(NeutralModeValue.Coast);
        pivotIntake.setNeutralMode(NeutralModeValue.Brake);


        selectIntakeState.setDefaultOption("NONE", IntakeStates.NONE);
        selectIntakeState.addOption("STOWED", IntakeStates.STOWED);
        selectIntakeState.addOption("INTAKE", IntakeStates.INTAKE);
        SmartDashboard.putData(selectIntakeState);

        SmartDashboard.putData("STATES[IN]", selectIntakeState);
        ShuffleboardPIDTuner.addSlider("PIVOT KP [IN]", 0.0, 1, 0.0);
        ShuffleboardPIDTuner.addSlider("PIVOT KD [IN]", 0.0, 0.05, 0.0);
    }

    @Override
    public void periodic() {
        // TODO: REMOVE - TEMP
        setState(selectIntakeState.getSelected());
        profiledPIDController.setP(ShuffleboardPIDTuner.get("PIVOT KP [IN]"));
        profiledPIDController.setD(ShuffleboardPIDTuner.get("PIVOT KD [IN]"));


        switch (state) {
            case NONE:
                pivotIntake.set(0);
                rollIntake.set(0);
                break;

            case STOWED:
                profiledPIDController.setGoal(Constants.Intake.kPivotAngleStowed);
                pivotIntake.set(profiledPIDController.calculate(getPivotAngle()));

                rollIntake.set(Constants.Intake.kRollSpeedStowed);
                break;

            case INTAKE:
                profiledPIDController.setGoal(Constants.Intake.kPivotAngleIntake);
                pivotIntake.set(profiledPIDController.calculate(getPivotAngle()));

                rollIntake.set(Constants.Intake.kRollSpeedIntake);
                break;
            
        }

        // clearStickyFaults();
        lastState = state;

        SmartDashboard.putNumber("Pivot Angle [IN]", getPivotAngle());
        SmartDashboard.putBoolean("At Goal [IN]", atGoal());
    }

    
    public void setState(IntakeStates state) {
        this.state = state;
    }

    public IntakeStates getState() {
        return state;
    }

    public double getPivotAngle() {
        double deg = Conversions.falconToDegrees(pivotIntake.getRotorPosition().getValueAsDouble(), Constants.Intake.kPivotIntakeGearRatio);
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
       return rollIntake.getStatorCurrent().getValueAsDouble() > Constants.Intake.kHasNoteCurrentThreshold;
    }

  public boolean atGoal() {
    double goal = profiledPIDController.getGoal().position;
    return Util.inRange(getPivotAngle(), (goal - Constants.Intake.kAtGoalTolerance), (goal + Constants.Intake.kAtGoalTolerance));
  }

  public void clearStickyFaults() {
    pivotIntake.clearStickyFaults();
    rollIntake.clearStickyFaults();
  }
}