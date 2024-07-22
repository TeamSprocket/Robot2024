// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//phoenix imports for pivot intake
import com.ctre.phoenix6.configs.FeedbackConfigs; // configs for feedback settings
import com.ctre.phoenix6.configs.MotionMagicConfigs; //configs for Motion Magic settings
import com.ctre.phoenix6.configs.MotorOutputConfigs; //configs for motor output settings
import com.ctre.phoenix6.configs.Slot0Configs; //configs class for Slot 0 settings
import com.ctre.phoenix6.configs.TalonFXConfiguration; //configs for Talon FX motor controller settings
import com.ctre.phoenix6.hardware.TalonFX; // Talon FX motor controller
import com.ctre.phoenix6.signals.GravityTypeValue; // signal value enum for GravityType
import com.ctre.phoenix6.signals.InvertedValue; // signal value enum for InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue; // signal value enum for NeutralModeValue
import com.ctre.phoenix6.controls.MotionMagicVoltage; // control class for Motion Magic voltage settings
import com.ctre.phoenix6.controls.VoltageOut; // control class for VoltageOut settings
import com.ctre.phoenix6.configs.CANcoderConfiguration; // Configures CANCoder
import com.ctre.phoenix6.configs.CurrentLimitsConfigs; // Limits the current that goes into the intake
import com.ctre.phoenix6.configs.TalonFXConfiguration; // Configures the Motor
import com.ctre.phoenix6.hardware.TalonFX; // Motor stuff
import com.ctre.phoenix6.signals.NeutralModeValue; // state of motor controller bridge where Intake is neutral or disabled

import edu.wpi.first.math.trajectory.TrapezoidProfile; // trajectory planning stuff
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // to choose different states on SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // to interact with SmartBoard
import edu.wpi.first.math.controller.PIDController; // PID controller
import edu.wpi.first.math.controller.ProfiledPIDController; // PID controller with setpoints

//spark max imports for roll intake
import com.revrobotics.CANSparkMax; //CAN SPeark MAX motor controller
import com.revrobotics.CANSparkBase.IdleMode; // definese idle mods for Spark Max
import com.revrobotics.CANSparkLowLevel.MotorType; //defining motor types for Spark Max

import edu.wpi.first.wpilibj2.command.SubsystemBase; // Base class for all subsystems
import frc.robot.Constants; // Constants specificfor robot intake (subsystem)
import frc.robot.RobotMap; // maps hardware ports
import frc.util.Conversions; // utility functions for unit consersions
import frc.util.ShuffleboardIO; // utitlity class for Shuffleboard
import frc.util.Util; // general untility functions


import com.ctre.phoenix6.motorcontrol.NeutralMode; // motor controller state where Intake is neutral or disabled
import com.ctre.phoenix6.motorcontrol.TalonFXConfiguration;
import com.ctre.phoenix6.motorcontrol.can.TalonFX;

public class IntakePivot extends SubsystemBase {

    // Motor controllers for roll and pivot intake
    private final TalonFX rollIntake = new TalonFX(RobotMap.Intake.ROLL_INTAKE);
    private final Talo\nFX pivotIntake = new TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    private double pivotSpeed = 0;

    // ProfiledPIDController profiledPIDController;
    // Current state and last state of the intake
    private IntakeStates state = IntakeStates.NONE;
    private IntakeStates lastState = IntakeStates.NONE;

     // SendableChooser for selecting intake states
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

    // MotionMagicVoltage and VoltageOut instances
    MotionMagicVoltage mmV = new MotionMagicVoltage(0);
    VoltageOut vO = new VoltageOut(0);

    public Intake() {
        configMotors();
        //pivot intake motor controller
        TalonFXConfiguration pivotIntakeConfig = new TalonFXConfiguration();
        
        pivotIntakeConfig.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Intake.kPivotIntakeMMCruiseVelocity)
                .withMotionMagicAcceleration(Constants.Intake.kPivotIntakeMMCruiseAccel)
        );

        pivotIntakeConfig.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.Intake.kPivotIntakeS) 
                .withKV(Constants.Intake.kPivotIntakeV) 
                .withKA(Constants.Intake.kPivotIntakeA) 
                .withKG(Constants.Intake.kPivotIntakeG) 
                .withKP(Constants.Intake.kPivotIntakeP)
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
         // configs 4 pivot intake motor controller
        pivotIntake.getConfigurator().apply(pivotIntakeConfig);
        mmV.Slot = 0;

        rollIntake.setInverted(Constants.Intake.kIsRollInverted);
        pivotIntake.setInverted(Constants.Intake.kIsPivotInverted);
        rollIntake.optimizeBusUtilization();

        Setting initial position for pivot intake
        pivotIntake.setPosition();

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
        // setState(selectIntakeState.getSelected());
        // pidController.setP(ShuffleboardIO.getDouble("PIVOT KP [IN]"));
        // pidController.setD(ShuffleboardIO.getDouble("PIVOT KD [IN]"));
        switch (state) {
            case NONE:
                pivotIntake.set(0);
                rollIntake.set(0);
                break;
            //all the way up
            case STOWED:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleStowed));
                rollIntake.setControl(vO.withOutput(0.0));
            // position intake
            case INTAKE:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleIntake));
                rollIntake.setControl(vO.withOutput(3.0));
                break;
            // indexing position stop roller 
            case INDEXING:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleIndexing));
                rollIntake.set(0.0);
                break;
            //undo the intake 
            case INTAKE_ROLLBACK:
                // Pivot maintains current position
                rollIntake.set(-1.0 * Constants.Intake.kRollSpeedIntakeRollback);
                break;
            //
            case SCORE_SPEAKER_SUBWOOFER:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleScoreSpeakerSubwoofer));
                rollIntake.set(0.0);
                break;

            // from anywhere
            case SCORE_SPEAKER:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAngleScoreSpeaker));
                rollIntake.set(Constants.Intake.kRollSpeedScoreSpeaker);
                break;

            case AMP:
                pivotIntake.setControl(mmV.withPosition(Constants.Intake.kPivotAnglesScoreAmp));
                rollIntake.set(pivotSpeed);
                break;
            

            // shoot note across field
            case CROSSFIELD:
                pivotIntake.setControl(mmv.withPosition(Constants.Intake.kPivotAngleShootCrossfield));
                rollIntake.set(0.0);
                // pidController.setSetpoint(Constants.Intake.kPivotAngleShootCrossfield);
                // pivotSpeed = pidController.calculate(getPivotAngle());
                // if (pidController.atSetpoint()) {
                //     pivotSpeed = 0.0;
                // }
                // pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Intake.kMaxPivotOutput, Constants.Intake.kMaxPivotOutput);
                // pivotIntake.set(pivotSpeed);
                // rollIntake.set(0.0);
                // break;

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
        SmartDashboard.putNumber("Pivot Angle Target [IN]", pidController.getSetpoint());
        SmartDashboard.putString("State Intake [IN]", state.toString());
    
    }

    public void setState(IntakeStates state) {
        this.state = state;
    }

    public IntakeStates getState() {
        return state;
    }

    public double getPivotSpeed(double targetAngle) {
        public double getPivotSpeed() {
        return pivotIntake.getVelocity().getValueAsDouble();
    }


    }
    //     pidController.setSetpoint(targetAngle);
    //     double currentAngle = getPivotAngle();

    //     double pivotSpeed;
    //     double PIDOutput = pidController.calculate(currentAngle); 
    //     // stops if PID too much 
    //     if (Math.abs(targetAngle - currentAngle) > Constants.Intake.kFFtoPIDPivotTransitionTolerance) {
    //         pivotSpeed = Constants.Intake.kFFPivot * Util.getSign(PIDOutput);
    //     } else {
    //         pivotSpeed = PIDOutput;
    //         //if it is at the point
    //         if (pidController.atSetpoint()) {
    //             pivotSpeed = 0;
    //         }
    //     }

    //     pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Intake.kMaxPivotOutput, Constants.Intake.kMaxPivotOutput);
    //     return pivotSpeed;
    // }
    // /**
    //  * gets currrent pivot angle 
    //  * @return
    //  */
    public double getPivotAngle() {
        double deg = -1 * Conversions.falconToDegrees(pivotIntake.getRotorPosition().getValueAsDouble(), Constants.Intake.kPivotIntakeGearRatio);
        deg = deg + Constants.Intake.kPivotAngleOffsetHorizontal;
        deg %= 360;
        deg = (deg < 0) ? deg + 360 : deg; 
        deg = (deg > 270) ? 0 : deg;
        return deg;

  public boolean atGoal() {
    double goal = mmV.Position;
    return Util.inRange(getPivotAngle(), (goal - Constants.Intake.kAtGoalTolerance), (goal + Constants.Intake.kAtGoalTolerance));
  }

  public void zeroPosition() {
    pivotIntake.setPosition(0);
    rollIntake.setPosition(0);
  }

  public void clearStickyFaults() {
    pivotIntake.clearStickyFaults();
    rollIntake.clearStickyFaults();
  }

  private void configMotors() {
    CurrentLimitsConfigs currentLimitsConfigsPivot = new CurrentLimitsConfigs();
    currentLimitsConfigsPivot.withSupplyCurrentLimit(Constants.Intake.kSupplyCurrentLimitPivot);
    currentLimitsConfigsPivot.withSupplyCurrentLimitEnable(true);
    CurrentLimitsConfigs currentLimitsConfigsRoll = new CurrentLimitsConfigs();
    currentLimitsConfigsRoll.withSupplyCurrentLimit(Constants.Intake.kSupplyCurrentLimitRoll);
    currentLimitsConfigsRoll.withSupplyCurrentLimitEnable(true);

    TalonFXConfiguration motorConfigPivot = new TalonFXConfiguration();
    // motorConfigPivot.withCurrentLimits(currentLimitsConfigsPivot);
    TalonFXConfiguration motorConfigRoll = new TalonFXConfiguration();
    // motorConfigRoll.withCurrentLimits(currentLimitsConfigsRoll);

    pivotIntake.getConfigurator().apply(motorConfigPivot);
    rollIntake.getConfigurator().apply(motorConfigRoll);
  }
}

