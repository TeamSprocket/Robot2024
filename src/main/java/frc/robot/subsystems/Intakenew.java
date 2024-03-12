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

public class Intakenew extends SubsystemBase{
    public enum IntakeStates {
        NONE,
        STOWED,
        INTAKENOTE,
        EJECTNOTE,
        ROLLBACK,
        PIVOTUP,
        PIVOTDOWN
    }

    private final TalonFX intakeMotor = new TalonFX(RobotMap.Intake.ROLL_INTAKE);
    private final TalonFX pivotMotor = new TalonFX(RobotMap.Intake.PIVOT_INTAKE);

    private PIDController pIDController = new PIDController(0, 0, 0);

    SendableChooser<IntakeStates> stateChooser = new SendableChooser<IntakeStates>();

    
}
