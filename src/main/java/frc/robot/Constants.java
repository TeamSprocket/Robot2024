package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.util.PIDConst;

public class Constants {
    public static enum RobotState {
        TELEOP,
        AUTON,
        DISABLED,
        TELEOP_DISABLE_SWERVE,
        LOCK_TURN_TO_APRIL_TAG
    }

    // Global
    public static RobotState robotState = RobotState.DISABLED;

    public static final class FieldConstants {

        // in meters
        public static final double kFieldLength = 16.54;
        public static final double kSpeakerY = 5.55;
        public static final double kSpeakerTargetHeightMeters = 2.032;
        public static final double kSpeakerAprilTagHeightMeters = 1.451;
    }

    public static final class Vision {

        public static final Matrix<N3, N1> kMultipleTagStdDevs = VecBuilder.fill(0.5, 0.5, 6.24);

        // measurements
        public static final Translation3d targetPointBlue = new Translation3d(0.0, 5.448, 0); // 5.548  // Moved towards right red alliance 0.1m
        public static final Translation3d targetPointRed = new Translation3d(0.0, 2.563, 0); //2.663
        
        public static final double kLimelightPitchAngleDegrees = 5.0;
        public static final double kLimelightHeightMeters = 0.534;

        public static final double kAcceptableVolatilityThreshold = 0.2;

        public static final double kMaxDrivingSpeed = 0.0;
        public static final double kMaxTurningSpeed = 0.1;

        public static final int kVolatilitySlidingWindowLen = 20;

        public static final double kdistanceOffset = 0.7642; // 0.5992

        // Most likely can be same blue/red
        // public static final Translation3d targetPoint = new Translation3d(-0.1, (8.211 - 5.548), 0);

        public static final double kShooterVelocityLinearMultiplier = 1.5;
        public static final double kShooterVelocityBase = 17.0;
    }

    public static final class Drivetrain {

        // Measurements
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kModuleOffsetMeters = 0.572;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( // TODO: check these just in case
                new Translation2d(-kModuleOffsetMeters / 2, kModuleOffsetMeters / 2), // -, -
                new Translation2d(-kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2), // -, +
                new Translation2d(kModuleOffsetMeters / 2, kModuleOffsetMeters / 2), // +, -
                new Translation2d(kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2)); // +, +


        public static final double kHeadingLockPIDMaxOutput = 2.0;
        public static final double kHeadingLockDegreeRejectionTolerance = 10.0;
        public static final double kHeadingLockDegreeTolerance = 1.0;

        public static double MaxSpeed = 4.73;
        public static double MaxAngularRate = 1.5 * Math.PI;

        // PID
        public static double kPTurnMotorFL = 0.00255; // 0.00255
        public static double kITurnMotorFL = 0.00;
        public static double kDTurnMotorFL = 0.000025; // 0.000025

        public static double kPTurnMotorFR = 0.0027; // 0.0027
        public static double kITurnMotorFR = 0.00;
        public static double kDTurnMotorFR = 0.0000; // 0.0

        public static double kPTurnMotorBL = 0.00255; // 0.00255
        public static double kITurnMotorBL = 0.00;
        public static double kDTurnMotorBL = 0.000025; // 0.000025

        public static double kPTurnMotorBR = 0.00255; // 0.00255
        public static double kITurnMotorBR = 0.00;
        public static double kDTurnMotorBR = 0.000025; // 0.000025

        public static PIDConst kPIDSpeakerHeadingLock = new PIDConst(0.2, 0, 0); // TODO: tune PID note heading lock is in radians while heading controller is in degrees

        public static final double kPTranslationPP = 0.25; // 3.0
        public static final double kITranslationPP = 0.0;
        public static final double kDTranslationPP = 0.0;

        public static final double kPRotationPP = 3.0; // 3.0
        public static final double kIRotationPP = 0.0;
        public static final double kDRotationPP = 0.0; // 0.0

        public static final double kTranslationMultPP = 0.2;

        public static final double kTurnFF = 0.12;
        public static final double kTurnPIDTolerance = 30;

        // Speed/Accel
        public static final double kMaxDriveModuleSpeedMPS = 4.0;

        public static double kMaxSpeed = 0.9; // 0.8 //1.0 // 0.6 - latest
        public static double kMaxAccel = 1.6; // 1.0 // 1.9 

        public static double kMaxTurnSpeed = 0.95; // 0.2 //1.1 // 0.08 WITH PID, 1.35 WITHOUT
        public static double kMaxTurnAccel = 7.0; // 1.5; // Instant manual turning

        // Misc
        public static final double kDrivingMotorDeadband = 0.05;

        public static final double kIntakeNoteSpeed = -0.3;

        public static final boolean kIsFieldOriented = true;

        public static boolean isPrecise = false;
        public static final double kPreciseMultiplier = 0.25;

        public static final double kTurnCurrentLimit = 100;
        public static final double kDriveCurrentLimit = 100;

        public static final InvertedValue FRONT_LEFT_D_IS_REVERSED = InvertedValue.CounterClockwise_Positive; // TODO: tune these :D
        public static final InvertedValue FRONT_RIGHT_D_IS_REVERSED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue BACK_LEFT_D_IS_REVERSED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue BACK_RIGHT_D_IS_REVERSED = InvertedValue.CounterClockwise_Positive;
        
        public static final InvertedValue FRONT_LEFT_T_IS_REVERSED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue FRONT_RIGHT_T_IS_REVERSED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue BACK_LEFT_T_IS_REVERSED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue BACK_RIGHT_T_IS_REVERSED = InvertedValue.CounterClockwise_Positive;

        // in rotations
        public static double kCANCoderOffsetFrontLeft = -0.055908; // TODO: check these too
        public static double kCANCoderOffsetFrontRight = -0.098389;
        public static double kCANCoderOffsetBackLeft = -0.357178;
        public static double kCANCoderOffsetBackRight = -0.013916; 
    }
}
