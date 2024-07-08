// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//phoenix imports for pivot intake
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//spark max imports for roll intake
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.ShuffleboardIO;
import frc.util.Util;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private final TalonFX rollIntake = new TalonFX(RobotMap.Intake.ROLL_INTAKE);
    private final TalonFX pivotIntake = new TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    private double pivotSpeed = 0;

    private IntakeStates state = IntakeStates.NONE;
    private IntakeStates lastState = IntakeStates.NONE;

    SendableChooser<IntakeStates> selectIntakeState = new SendableChooser<IntakeStates>();

    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKE,
        INTAKE_ROLLBACK,
        INDEXING,
        SCORE_SPEAKER_SUBWOOFER, 
        SCORE_SPEAKER,
        AMP,
        CROSSFIELD,
        EJECT_NOTE,
        CLIMB
    }

    MotionMagicVoltage mmV = new MotionMagicVoltage(0);
    VoltageOut vO = new VoltageOut(0);

    public Intake() {
        // configMotors();

        TalonFXConfiguration pivotIntakeConfig = new TalonFXConfiguration();
        
        pivotIntakeConfig.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Intake.kPivotIntakeMMCruiseVelocity)
                .withMotionMagicAcceleration(Constants.Intake.kPivotIntakeMMCruiseAccel)
        );

        pivotIntakeConfig.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.Intake.kPivotIntakeS) // 0.27
                .withKV(Constants.Intake.kPivotIntakeV) // 1.4
                .withKA(Constants.Intake.kPivotIntakeA) // 0.01
                .withKG(Constants.Intake.kPivotIntakeG) // 0.20
                .withKP(Constants.Intake.kPivotIntakeP) // 2
                .withKI(Constants.Intake.kPivotIntakeI)
                .withKD(Constants.Intake.kPivotIntakeD)
        );

        pivotIntakeConfig.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Intake.kPivotIntakeGearRatio)
        );

        // pivotIntakeConfig.withMotorOutput(
        //     new MotorOutputConfigs()
        //         .withNeutralMode(NeutralModeValue.Brake)
        //         .withInverted(InvertedValue.CounterClockwise_Positive)
        // );
        // PRETTY SURE THESE DON'T MATTER BUT REENABLE IN CASE SOMETHING WRONG

        pivotIntake.getConfigurator().apply(pivotIntakeConfig);
        mmV.Slot = 0;

        rollIntake.setInverted(Constants.Intake.kIsRollInverted);
        pivotIntake.setInverted(Constants.Intake.kIsPivotInverted);
        rollIntake.optimizeBusUtilization();

        rollIntake.setNeutralMode(NeutralModeValue.Coast);
        pivotIntake.setNeutralMode(NeutralModeValue.Brake);

        pivotIntake.setPosition(0.2222);

        // selectIntakeState.setDefaultOption("NONE", IntakeStates.NONE);
        // selectIntakeState.addOption("STOWED", IntakeStates.STOWED);
        // selectIntakeState.addOption("INTAKE", IntakeStates.INTAKE);
        // SmartDashboard.putData(selectIntakeState);

        // SmartDashboard.putData("STATES[IN]", selectIntakeState);

        // ShuffleboardIO.addSlider("PIVOT KP [IN]", 0.0, 0.01, Constants.Intake.kPPivot);
        // ShuffleboardIO.addSlider("PIVOT KD [IN]", 0.0, 0.001, Constants.Intake.kDPivot);
        // ShuffleboardIO.addSlider("Intake Pivot Voltage [IN]", 0.0, 1.5, 0);
    }

    @Override
    public void periodic() {
        // debug - state selector

        // setState(selectIntakeState.getSelected()); // COMMENT OUT PLZ
        switch (state) {
            case NONE:
                pivotIntake.set(0);
                rollIntake.set(0);
                break;

            case STOWED:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleStowed));
                rollIntake.setControl(vO.withOutput(0.0));
                break;

            case INTAKE:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleIntake));
                rollIntake.setControl(vO.withOutput(3.0));
                break;

            case INDEXING:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleIndexing));
                rollIntake.set(0.0);
                break;

            case INTAKE_ROLLBACK:
                // Pivot maintains current position
                rollIntake.set(-1.0 * Constants.Intake.kRollSpeedIntakeRollback);
                break;
                
            case SCORE_SPEAKER_SUBWOOFER:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleScoreSpeakerSubwoofer));
                rollIntake.set(0.0);
                break;

            case SCORE_SPEAKER:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleScoreSpeaker));
                rollIntake.set(Constants.Intake.kRollSpeedScoreSpeaker);
                break;

            case EJECT_NOTE:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleEject));
                rollIntake.set(Constants.Intake.kEjectNoteSpeed);
                break;

            case CLIMB:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleClimb));
                rollIntake.set(0.0);
                break; 
        }

        lastState = state;

        SmartDashboard.putNumber("Pivot Angle [IN]", getPivotAngle());
        SmartDashboard.putBoolean("At Goal [IN]", atGoal());
        // SmartDashboard.putNumber("Pivot Angle Target [IN]", pidController.getSetpoint());
        SmartDashboard.putString("State Intake [IN]", state.toString());
        SmartDashboard.putNumber("Motion Magic Output [IN]", pivotIntake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roll Intake Output [IN]", rollIntake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Position [IN]", pivotIntake.getPosition().getValueAsDouble());

        // SmartDashboard.putString("Neutral Mode Value PivotIntake [IN]", );

        //debug
        // SmartDashboard.putString("State[IN]", state.toString());
        // SmartDashboard.putNumber("", pivotSpeed)
    }

    
    public void setState(IntakeStates state) {
        this.state = state;
    }

    public IntakeStates getState() {
        return state;
    }

    public double getPivotSpeed() {
        return pivotIntake.getVelocity().getValueAsDouble();
    }

    public double getPivotAngle() {
        double deg = -1 * Conversions.falconToDegrees(pivotIntake.getRotorPosition().getValueAsDouble(), Constants.Intake.kPivotIntakeGearRatio);
        deg = deg + Constants.Intake.kPivotAngleOffsetHorizontal;
        deg %= 360;
        deg = (deg < 0) ? deg + 360 : deg; 
        deg = (deg > 270) ? 0 : deg;
        return deg;
    }

  public boolean atGoal() {
    double goal = mmV.Position;
    return Util.inRange(getPivotAngle(), (goal - Constants.Intake.kAtGoalTolerance), (goal + Constants.Intake.kAtGoalTolerance));
  }

  public void zeroPosition() {
    pivotIntake.setPosition(0);
    rollIntake.setPosition(0);
  }

  public void setNeutralModePivot(NeutralModeValue neutralModeValue) {
    pivotIntake.setNeutralMode(neutralModeValue);
  } 

  public void clearStickyFaults() {
    pivotIntake.clearStickyFaults();
    rollIntake.clearStickyFaults();
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    pivotIntake.setNeutralMode(neutralModeValue);
  }

  private void configMotors() {
    // CurrentLimitsConfigs currentLimitsConfigsPivot = new CurrentLimitsConfigs();
    // currentLimitsConfigsPivot.withSupplyCurrentLimit(Constants.Intake.kSupplyCurrentLimitPivot);
    // currentLimitsConfigsPivot.withSupplyCurrentLimitEnable(true);
    // CurrentLimitsConfigs currentLimitsConfigsRoll = new CurrentLimitsConfigs();
    // currentLimitsConfigsRoll.withSupplyCurrentLimit(Constants.Intake.kSupplyCurrentLimitRoll);
    // currentLimitsConfigsRoll.withSupplyCurrentLimitEnable(true);

    TalonFXConfiguration motorConfigPivot = new TalonFXConfiguration();
    // motorConfigPivot.withCurrentLimits(currentLimitsConfigsPivot);
    TalonFXConfiguration motorConfigRoll = new TalonFXConfiguration();
    // motorConfigRoll.withCurrentLimits(currentLimitsConfigsRoll);

    pivotIntake.getConfigurator().apply(motorConfigPivot);
    rollIntake.getConfigurator().apply(motorConfigRoll);
  }
}