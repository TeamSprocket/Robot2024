package frc.robot.subsystems;

// //phoenix imports for pivot intake
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
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

public class Shintake extends SubsystemBase{
    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKENOTE,
        EJECTNOTE,
        SCOREAMP
    }
    IntakeStates state;

    private final TalonFX intakeMotor = new TalonFX(RobotMap.Intake.ROLL_INTAKE);
    private final TalonFX pivotMotor = new TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    private double pivotSpeed = 0;

    private PIDController pidControllerIntake;
    private PIDController pidControllerShooter;

    SendableChooser<IntakeStates> stateChooser = new SendableChooser<IntakeStates>();

    //constructor
    public Shintake() {
        pidControllerIntake = new PIDController(Constants.Intake.kPPivot, Constants.Intake.kIPivot, Constants.Intake.kDPivot);
        intakeMotor.setInverted(false);
        pivotMotor.setInverted(true);

        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);

        stateChooser.setDefaultOption("NONE", IntakeStates.NONE);
        stateChooser.addOption("STOWED", IntakeStates.STOWED);
        stateChooser.addOption("INTAKENOTE", IntakeStates.INTAKENOTE);
        stateChooser.addOption("EJECTNOTE", IntakeStates.EJECTNOTE);
        stateChooser.addOption("SCOREAMP", IntakeStates.SCOREAMP);

        SmartDashboard.putData("Intake State", stateChooser);
    }

    @Override
    public void periodic() {
        switch(state) {
            case NONE :
                intakeMotor.set(0);
                pivotMotor.set(0);
                break;
            
            case STOWED :
                pidControllerIntake.setSetpoint(Constants.Shintake.kPivotAngleStowed);
                pivotSpeed = pidControllerIntake.calculate(getDegrees());
                pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Shintake.kMaxPivotOutput, Constants.Shintake.kMaxPivotOutput);
            
                intakeMotor.set(0);
                pivotMotor.setPosition(pivotSpeed);
                break;

            case INTAKENOTE :
                pidControllerIntake.setSetpoint(Constants.Shintake.kPivotAngleIntake);
                pivotSpeed = pidControllerIntake.calculate(getDegrees());
                pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Shintake.kMaxPivotOutput, Constants.Shintake.kMaxPivotOutput);

                intakeMotor.set(Constants.Shintake.kRollSpeedIntake);
                pivotMotor.setPosition(pivotSpeed);
                break;
            
            case EJECTNOTE :
                pidControllerIntake.setSetpoint(Constants.Shintake.kPivotAngleIntake);
                pivotSpeed = pidControllerIntake.calculate(getDegrees());
                pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Shintake.kMaxPivotOutput, Constants.Shintake.kMaxPivotOutput);

                intakeMotor.set(-1 * Constants.Shintake.kRollSpeedIntake);
                pivotMotor.setPosition(pivotSpeed);
                break;
            
            case SCOREAMP :
                pidControllerIntake.setSetpoint(Constants.Shintake.kScoreAmp);
                pivotSpeed = pidControllerIntake.calculate(getDegrees());
                pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.Shintake.kMaxPivotOutput, Constants.Shintake.kMaxPivotOutput);

                intakeMotor.set(Constants.Shintake.kRollSpeedScoreAmp);
                pivotMotor.set(pivotSpeed);
                break;
        }
    }

    public IntakeStates getState() {
        return state;
    }
    public void setState(IntakeStates state) {
        this.state = state;
    }

    public double getDegrees() {
        double deg = Conversions.falconToDegrees(pivotMotor.getRotorPosition().getValueAsDouble(), Constants.Intake.kPivotIntakeGearRatio);
        deg = deg % 360;
        return deg;
    }
}

