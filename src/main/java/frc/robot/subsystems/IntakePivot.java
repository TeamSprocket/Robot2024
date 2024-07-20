// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//phoenix imports for pivot intake
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

/** Add your docs here. */
public class Intake extends SubsystemBase {

    // stuff for motor roll and pivot intake
    private final CANSparkMax rollIntake = new CANSparkMax(RobotMap.Intake.ROLL_INTAKE);
    private final CANSparkMax pivotIntake = new CANSparkMax(RobotMap.Intake.PIVOT_INTAKE);

    private double pivotSpeed = 0; // speed variable for pivot intake


    // ProfiledPIDController profiledPIDController;
    private PIDController pidController;

    private IntakeStates state = IntakeStates.NONE; //current intake state
    private IntakeStates lastState = IntakeStates.NONE; // the last intake state

    SendableChooser<IntakeStates> selectIntakeState = new SendableChooser<IntakeStates>(); //chooses intake states on SmartDashboard

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


    public Intake() {
        configMotors();

        pidController = new PIDController(Constants.Intake.kPPivot, 0, Constants.Intake.kDPivot);

        selectIntakeState.setDefaultOption("NONE", IntakeStates.NONE);
        selectIntakeState.addOption("STOWED", IntakeStates.STOWED);
        selectIntakeState.addOption("INTAKE", IntakeStates.INTAKE);

        rollIntake.setInverted(Constants.Intake.kIsRollInverted);
        pivotIntake.setInverted(Constants.Intake.kIsPivotInverted);
        rollIntake.optimizeBusUtilization();

        //default state
        //this allow motor to move and roll motor to move
        rollIntake.setNeutralMode(NeutralModeValue.Coast);
        //this makes the pivot not move
        pivotIntake.setNeutralMode(NeutralModeValue.Brake);

        //
        selectIntakeState.setDefaultOption("NONE", IntakeStates.NONE);
        selectIntakeState.addOption("STOWED", IntakeStates.STOWED);
        selectIntakeState.addOption("INTAKE", IntakeStates.INTAKE);
//        SmartDashboard.putData(selectIntakeState);

        SmartDashboard.putData("STATES[IN]", selectIntakeState);

        ShuffleboardIO.addSlider("PIVOT KP [IN]", 0.0, 0.01, Constants.Intake.kPPivot);
        ShuffleboardIO.addSlider("PIVOT KD [IN]", 0.0, 0.001, Constants.Intake.kDPivot);
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
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleStowed);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(0.0);
                break;  
            // position intake
            case INTAKE:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleIntake);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(Constants.Intake.kRollSpeedIntake);
                break;
            // indexing position stop roller 
            case INDEXING:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleIndexing);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(0.0);
                break;
            //undo the intake 
            case INTAKE_ROLLBACK:
                // Pivot maintains current position
                rollIntake.set(-1.0 * Constants.Intake.kRollSpeedIntakeRollback);
                break;
            //
            case SCORE_SPEAKER_SUBWOOFER:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleScoreSpeakerSubwoofer);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(0.0);
                break;
            // from anywhere
            case SCORE_SPEAKER:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleScoreSpeaker);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(0.0);
                break;

            case AMP:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleScoreAmp);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(0.0);
                break;
            

            // shoot note across field
            case CROSSFIELD:
                pidController.setSetpoint(Constants.Intake.kPivotAngleShootCrossfield);
                pivotSpeed = pidController.calculate(getPivotAngle());
                if (pidController.atSetpoint()) {
                    pivotSpeed = 0.0;
                }
                pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Intake.kMaxPivotOutput, Constants.Intake.kMaxPivotOutput);
                pivotIntake.set(pivotSpeed);
                rollIntake.set(0.0);
                break;

            case EJECT_NOTE:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleEject);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(Constants.Intake.kEjectNoteSpeed);
                break;

            case CLIMB:
                pivotSpeed = getPivotSpeed(Constants.Intake.kPivotAngleClimb);
                pivotIntake.set(pivotSpeed);

                rollIntake.set(0.0);
                break;

            
            
        }

        lastState = state;

        SmartDashboard.putNumber("Pivot Angle [IN]", getPivotAngle());
        SmartDashboard.putBoolean("At Goal [IN]", atGoal());
        SmartDashboard.putNumber("Pivot Angle Target [IN]", pidController.getSetpoint());
        SmartDashboard.putString("State Intake [IN]", state.toString());

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

    public double getPivotSpeed(double targetAngle) {
        pidController.setSetpoint(targetAngle);
        double currentAngle = getPivotAngle();

        double pivotSpeed;
        double PIDOutput = pidController.calculate(currentAngle); 
        // stops if PID too much 
        if (Math.abs(targetAngle - currentAngle) > Constants.Intake.kFFtoPIDPivotTransitionTolerance) {
            pivotSpeed = Constants.Intake.kFFPivot * Util.getSign(PIDOutput);
        } else {
            pivotSpeed = PIDOutput;
            //if it is at the point
            if (pidController.atSetpoint()) {
                pivotSpeed = 0;
            }
        }

        pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Intake.kMaxPivotOutput, Constants.Intake.kMaxPivotOutput);
        return pivotSpeed;
    }
    /**
     * gets currrent pivot angle 
     * @return
     */
    public double getPivotAngle() {
        double deg = Conversions.falconToDegrees(pivotIntake.getRotorPosition().getValueAsDouble(), Constants.Intake.kPivotIntakeGearRatio);
        deg %= 360;
        deg = (deg < 0) ? deg + 360 : deg; 
        deg = (deg > 270) ? 0 : deg;
        return deg;
    }

  public boolean atGoal() {
    double goal = pidController.getSetpoint();
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