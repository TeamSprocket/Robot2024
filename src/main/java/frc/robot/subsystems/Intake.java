// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//phoenix imports for pivot intake
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
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

    private double pivotSpeed = 0;

    MotionMagicVoltage mmV = new MotionMagicVoltage(0);

    // ProfiledPIDController profiledPIDController;
    //PIDController pidController;

    private IntakeStates state = IntakeStates.STOWED;
    private IntakeStates lastState = IntakeStates.NONE;

    SendableChooser<IntakeStates> selectIntakeState = new SendableChooser<IntakeStates>();

    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKE,
        INTAKE_ROLLBACK,
        SCORE_SPEAKER, 
        EJECT_NOTE
    }


    public Intake() {
        // TrapezoidProfile.Constraints pivotProfileConstraints = new TrapezoidProfile.Constraints(Constants.Intake.kPivotMaxVelocity, Constants.Intake.kPivotMaxAccel);
        // profiledPIDController = new ProfiledPIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot, pivotProfileConstraints);

        rollIntake.setInverted(Constants.Intake.kIsRollInverted);
        pivotIntake.setInverted(Constants.Intake.kIsPivotInverted);
        rollIntake.optimizeBusUtilization();

        rollIntake.setNeutralMode(NeutralModeValue.Coast);
        pivotIntake.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration pivotIntakeConfig = new TalonFXConfiguration();
        
        pivotIntakeConfig.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(5)
                .withMotionMagicAcceleration(5)
        );

        pivotIntakeConfig.withSlot0(
            new Slot0Configs()
                .withKS(0.25)
                .withKV(0.1)
                .withKA(0.01)
                .withKP(Constants.Intake.kPPivot)
                .withKI(Constants.Intake.kIPivot)
                .withKD(Constants.Intake.kDPivot)
        );

        pivotIntakeConfig.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Intake.kPivotIntakeGearRatio)
        );

        pivotIntake.getConfigurator().apply(pivotIntakeConfig);

        selectIntakeState.setDefaultOption("NONE", IntakeStates.NONE);
        selectIntakeState.addOption("STOWED", IntakeStates.STOWED);
        selectIntakeState.addOption("INTAKE", IntakeStates.INTAKE);
        SmartDashboard.putData(selectIntakeState);

        SmartDashboard.putData("STATES[IN]", selectIntakeState);

        ShuffleboardPIDTuner.addSlider("PIVOT KP [IN]", 0.0, 0.01, Constants.Intake.kPPivot);
        ShuffleboardPIDTuner.addSlider("PIVOT KD [IN]", 0.0, 0.001, Constants.Intake.kDPivot);
    }

    @Override
    public void periodic() {
        // TODO: REMOVE - TEMP
        // setState(selectIntakeState.getSelected());
        // pidController.setP(ShuffleboardPIDTuner.get("PIVOT KP [IN]"));
        // pidController.setD(ShuffleboardPIDTuner.get("PIVOT KD [IN]"));


        switch (state) {
            case NONE:
                pivotIntake.set(0);
                rollIntake.set(0);
                break;

            case STOWED:
                //temporary positions, will create constants after testing
                pivotIntake.setControl(mmV.withPosition(0.01));

                rollIntake.set(0.0);
                break;

            case INTAKE:
                pivotIntake.setControl(mmV.withPosition(0.02));

                rollIntake.set(Constants.Intake.kRollSpeedIntake);
                break;

            case INTAKE_ROLLBACK:
                // Pivot maintains current pos

                rollIntake.set(-1.0 * Constants.Intake.kRollSpeedIntakeRollback);
                break;
                
            case SCORE_SPEAKER:
                pivotIntake.setControl(mmV.withPosition(0.03));

                rollIntake.set(Constants.Intake.kRollSpeedScoreSpeaker);
                break;

            case EJECT_NOTE:
                pivotIntake.setControl(mmV.withPosition(0.04));

                rollIntake.set(Constants.Intake.kEjectNoteSpeed);
                break;
            
        }

        // clearStickyFaults();
        lastState = state;

        // SmartDashboard.putNumber("Pivot Angle [IN]", getPivotAngle());
        // SmartDashboard.putBoolean("At Goal [IN]", atGoal());
        // SmartDashboard.putNumber("Pivot Angle Target [IN]", pidController.getSetpoint());
        // SmartDashboard.putString("State[IN]", state.toString());

        // SmartDashboard.putNumber("", pivotSpeed)
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
        deg = (deg > 270) ? 0 : deg;
        return deg;
    }

    // public void runPivotToSetpoint(double setpoint){
    //     /*Don't know which calculate methods to use */
    //     //setpoint = pivotProfile.calculate(0.02, setpoint, goal);
    //     double output = pivotPIDProfiled.calculate(getPivotAngle(), setpoint); 
    //     pivotIntake.setVoltage(output + pivotFeedForward.calculate(pivotPIDProfiled.getSetpoint().velocity, 0)); 
    // }
    

  public boolean atGoal() {
    double goal = mmV.Position;
    return Util.inRange(getPivotAngle(), (goal - Constants.Intake.kAtGoalTolerance), (goal + Constants.Intake.kAtGoalTolerance));
  }

  public void clearStickyFaults() {
    pivotIntake.clearStickyFaults();
    rollIntake.clearStickyFaults();
  }
}