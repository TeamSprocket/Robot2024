package frc.robot.subsystems;

// //phoenix imports for pivot intake
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

public class Shintake extends SubsystemBase {
    public enum ShintakeStates {
        NONE,
        STOWED,
        INTAKE_NOTE, // TODO: separate the words w/ underscores, snake case (INTAKE_NOTE)
        EJECT_NOTE,
        SCORE_AMP,
        WAIT_AMP
    }

    ShintakeStates state; // TODO:get rid of none

    private final CANSparkMax shintakeMotor = new CANSparkMax(RobotMap.Shintake.SHINTAKE, MotorType.kBrushless); 
    private final TalonFX indexerMotor = new TalonFX(RobotMap.Shintake.INDEXER); 
    private final TalonFX pivotMotor = new TalonFX(RobotMap.Shintake.PIVOT);

    // private double pivotSpeed = 0;

    private PIDController pidControllerIntake;
    private PIDController pidControllerShooter;

    SendableChooser<ShintakeStates> stateChooser = new SendableChooser<ShintakeStates>();

    // constructor
    public Shintake() { // TODO: In Constants.java, in Shintake, remove every variable not being used,
                        // and add as needed later, theres waaaaay too many rn and most of them are
                        // redundant
        pidControllerIntake = new PIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot,
                Constants.Intake.kDPivot);
        shintakeMotor.setInverted(true);
        pivotMotor.setInverted(true);
        indexerMotor.setInverted(true);

        shintakeMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        zeroPivot();

        stateChooser.setDefaultOption("NONE", ShintakeStates.NONE);
        stateChooser.addOption("STOWED", ShintakeStates.STOWED);
        stateChooser.addOption("INTAKE_NOTE", ShintakeStates.INTAKE_NOTE);
        stateChooser.addOption("EJECT_NOTE", ShintakeStates.EJECT_NOTE);
        stateChooser.addOption("SCORE_AMP", ShintakeStates.SCORE_AMP);
        stateChooser.addOption("WAITAMP", ShintakeStates.WAIT_AMP);


        SmartDashboard.putData("Intake State", stateChooser);
        
        ShuffleboardPIDTuner.addSlider("PID kP pivot", 0, 1, 0);
        ShuffleboardPIDTuner.addSlider("PID kD pivot", 0, 1, 0);
        
    }

    @Override
    public void periodic() {
        setState(stateChooser.getSelected());
        switch (state) {
            case NONE: // TODO: remove space before colons ;-;
                shintakeMotor.set(0);
                pivotMotor.set(0);
                break;

            case STOWED:
                shintakeMotor.set(Constants.Shintake.kRollSpeedStowed);
                pivotMotor.set(getPivotSpeed(Constants.Shintake.kPivotAngleStowed));
                break;
    
            case INTAKE_NOTE:
                shintakeMotor.set(Constants.Shintake.kRollSpeedIntake);
                indexerMotor.set(Constants.Shintake.kIndexerSpeedIntake);
                pivotMotor.set(getPivotSpeed(Constants.Shintake.kPivotAngleIntake));
                break;

            case EJECT_NOTE:
                shintakeMotor.set(-1 * Constants.Shintake.kRollSpeedIntake);
                indexerMotor.set(Constants.Shintake.kIndexerEjectNoteSpeed);
                pivotMotor.set(0);
                break;

            case SCORE_AMP:
                shintakeMotor.set(Constants.Shintake.kRollSpeedScoreAmp);
                indexerMotor.set(Constants.Shintake.kIndexerSpeedScoreAmp);
                pivotMotor.set(0);
                break;

            case WAIT_AMP:
                shintakeMotor.set(Constants.Shintake.kRollSpeedScoreAmp);
                indexerMotor.set(0);
                pivotMotor.set(getPivotSpeed(Constants.Shintake.kScoreAmp));
                break;
        }

        pidControllerIntake.setP(ShuffleboardPIDTuner.get("PID kP pivot"));
        pidControllerIntake.setD(ShuffleboardPIDTuner.get("PID kD pivot"));

        SmartDashboard.putNumber("Pivot Angle Position", getDegrees());
    }

    public ShintakeStates getState() {
        return state;
    }

    public void setState(ShintakeStates state) {
        this.state = state;
    }

    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        pivotMotor.setNeutralMode(neutralModeValue);
    }

    public double getDegrees() {
        double deg = Conversions.falconToDegrees(pivotMotor.getRotorPosition().getValueAsDouble(),
                Constants.Intake.kPivotIntakeGearRatio);
        deg = deg - Constants.Shintake.kPivotAngleOffset;
        deg = deg % 360;
        return deg;
    }

    public double getPivotSpeed(double setPoint) {
        double pivotSpeed = 0;
        pidControllerIntake.setSetpoint(setPoint);
        pivotSpeed = pidControllerIntake.calculate(getDegrees());
        pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Shintake.kMaxPivotOutput,
                Constants.Shintake.kMaxPivotOutput);
        return pivotSpeed;
    }

    public boolean hasNote() {
        return (indexerMotor.getStatorCurrent().getValueAsDouble() > Constants.Shintake.kIndexerHasNoteIndexThreshold);
    }

    public CANSparkMax getShintakeMotor() {
        return shintakeMotor;
    }

    public void zeroPivot() {
        pivotMotor.setPosition(0);
    }

    // TODO: Overall looks really great, actually crazy progress for 1 day gjgj :)
}
