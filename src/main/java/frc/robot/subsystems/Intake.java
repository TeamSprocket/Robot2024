// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    private double intakeSpeed = 0;

    // ProfiledPIDController profiledPIDController;

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

    VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    final VoltageOut m_request = new VoltageOut(0);
    
    // VoltageOut vO = new VoltageOut(0);

    public Intake() {
        // configMotors();

        TalonFXConfiguration pivotIntakeConfig = new TalonFXConfiguration();
        
        pivotIntakeConfig.withSlot0(
            new Slot0Configs()
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

        TalonFXConfiguration rollIntakeConfig = new TalonFXConfiguration();

        rollIntakeConfig.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.Intake.kRollIntakeS) // voltage value: overcome static friction
                .withKV(Constants.Intake.kRollIntakeV) // voltage to maintain motor's velocity at 1 rmps
                .withKP(Constants.Intake.kRollIntakeP) // value to start oscillating
                .withKI(Constants.Intake.kRollIntakeI) 
                .withKD(Constants.Intake.kRollIntakeD) // value to stop oscillating
        );

        

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
                //rollIntake.setControl(vO.withOutput(0.0));
                rollIntake.setControl(velocityVoltage.withVelocity(Constants.Intake.kRollSpeedStowed));
                break;

            case INTAKE:
                // rollIntake.setControl(velocityVoltage.withVelocity(Constants.Intake.kRollSpeedIntake));
                rollIntake.setControl(m_request.withOutput(1.0));
                break;

            case INDEXING:
                rollIntake.set(0.0);
                 rollIntake.setControl(velocityVoltage.withVelocity(0));
                break;

            case INTAKE_ROLLBACK:
                // Pivot maintains current position
                rollIntake.set(-1.0 * Constants.Intake.kRollSpeedIntakeRollback);
                rollIntake.setControl(velocityVoltage.withVelocity(Constants.Intake.kRollSpeedIntake));
                break;
                
            case SCORE_SPEAKER_SUBWOOFER:
                rollIntake.setControl(velocityVoltage.withVelocity(Constants.Intake.kRollSpeedScoreSpeaker));
                break;

            case SCORE_SPEAKER:
                // rollIntake.set(Constants.Intake.kRollSpeedScoreSpeaker);
                rollIntake.setControl(velocityVoltage.withVelocity(Constants.Intake.kRollSpeedScoreSpeaker));
                break;

            case EJECT_NOTE:
                // rollIntake.set(Constants.Intake.kEjectNoteSpeed);
                rollIntake.setControl(velocityVoltage.withVelocity(Constants.Intake.kEjectNoteSpeed));
                break;

            case CLIMB:
                // rollIntake.set(0.0);
                rollIntake.setControl(velocityVoltage.withVelocity(0));
                break; 
        }

        lastState = state;

        SmartDashboard.putNumber("Pivot Angle [IN]", getPivotAngle());
        // SmartDashboard.putBoolean("At Goal [IN]", atGoal());
        // SmartDashboard.putNumber("Pivot Angle Target [IN]", pidController.getSetpoint());
        SmartDashboard.putString("State Intake [IN]", state.toString());
        // SmartDashboard.putNumber("Motion Magic Output [IN]", pivotIntake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roll Intake Output [IN]", rollIntake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Position [IN]", pivotIntake.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roll Intake Velocity [IN]", rollIntake.getRotorVelocity().getValueAsDouble());

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

//   public boolean atGoal() {
//     return Util.inRange(getPivotAngle(), (goal - Constants.Intake.kAtGoalTolerance), (goal + Constants.Intake.kAtGoalTolerance));
//   }

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