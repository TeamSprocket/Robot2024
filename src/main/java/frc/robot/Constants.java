package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.util.Conversions;
import frc.util.PIDConst;

public class Constants {
    public static enum RobotState {
        TELEOP,
        AUTON,
        DISABLED,
        TELEOP_DISABLE_SWERVE,
        TELEOP_LOCK_TURN_TO_SPEAKER
    }

    // Global
    public static RobotState robotState = RobotState.DISABLED;

    public static final class Superstructure {
        // Delay between note detected and retract intake
        public static final double kIntakeTimeToStowToleranceSec = 0.25;

        // Elapsed time beam break detecting note before stow
        public static final double kIndexerIntakeRollBackTimeSec = 0.05; // 0.2 // 0.15

        public static final double kIndexerIntakeRollForwardTimeSec = 0.3; // 0.15 //.0.05
        public static final double kIndexerIntakeRollForwardTimeSecPodium = 0.15;

        public static final double kRollForwardtoRollBackWaitTime = 0.4; // 0.4

        // Elapsed time after shooter hits speaker speed before score speaker
        public static final double kWaitSpeakerTimeToleranceSec = 0.3; 

        // Elapsed time shooting into speaker before stow
        public static final double kScoreSpeakerShootDurationSec = 1.5; // intake fallback = 0.5s,\1.0

        public static final double kScoreSpeakerPivotTimeToleranceSec = 0.3;

        public static final double kScoreAmpDuration = 1.5;

        public static final double kRumbleHasNoteTime = 0.5;

        //
        public static final double kScoreSpeakerHeadingTolerance = 0.25;
    }

    public static final class FieldConstants {

        // in meters
        public static final double kFieldLength = 16.54;
        public static final double kSpeakerY = 5.55;
        public static final double kSpeakerTargetHeightMeters = 2.032;
        public static final double kSpeakerAprilTagHeightMeters = 1.451;
    }

    public static final class Elevator {
        public static final double kSupplyCurrentLimit = 0.0;
        
        public static final double kElevatorHeightStowed = 0.005;
        public static final double kElevatorHeightAmp = 0.13; // 0.14 // 0.22; // 0.21;
        public static final double kElevatorHeightClimbUp = 0.18; // TODO: tune
        public static final double kElevatorHeightClimbDown = 0.005;


        // public static final double kElevatorHeightTrap = 0.0; 

        public static final PIDConst kPIDElevator = new PIDConst(2.15, 0.0, 0.0, 0.025);
        public static final double kElevatorVelocity = 0.35; // 0.25
        public static final double kElevatorMotorMaxOutput = 0.4;
        public static final double kElevatorMotorMaxOutputClimb = 0.2;

        public static final double kFFScaleWithHeight = 0.55;
        public static final double kFFBaseWithHeight = 0.015;
        
        
        public static final double kFFtoPIDTransitionToleranceM = 0.01;

        public static final double kManualMultiplier = 0.1;

        public static final double kAtGoalTolerance = 0.01;

        public static final double kElevatorGearRatio = 18.89;
        public static final double kElevatorWinchDiameterM = Conversions.inchesToMeters(1.5);
        public static final double kElevatorWinchCircumM = kElevatorWinchDiameterM * Math.PI;

        public static final boolean kLeftMotorIsInverted = true; // true
        public static final boolean kRightMotorIsInverted = false; // false

        public static final double kMaxVelocity = 0.03;
        public static final double kMaxAccel = 0.02;
    } 

    public static final class ShooterPivot {

        public static final double kSupplyCurrentLimit = 0.0;

        public static final boolean kIsShooterPivotInverted = false;

        public static PIDConst kPID = new PIDConst(0.0090, 0, 0, 0.015); // p: 0.00675

        public static final double kManualMultiplier = 0.001;

        public static final double kAtGoalTolerance = 0.0; // 0.25

        public static final double kShooterPivotGearRatio = 148.15;

        public static double kHorizontalAngle = 60.985;

        public static final double kMaxShooterPivotOutput = 0.55; // 0.55
        public static final double kMaxAngle = 90; // TODO: tune this

        public static final double kMaxDistance = 6.0;

        public static final double kFFtoPIDTransitionTolerance = 5;
        public static final double kFFPivot = 0.5;

        // public static final double kMaxVelocityDeg = 0.0;
        // public static final double kMaxAccelerationDeg = 0.0;

        public static final double kTargetAngleStowed = 6.0;
        public static final double kTargetAngleIntake = 13.5;
        public static final double kTargetAngleIndexing = 6.0;
        public static final double kTargetAngleEject = 60.0;
        // public static final double kTargetAngleSpeakerHigh = 0.0;
        public static final double kTargetAngleSpeakerFromSubwoofer = 9.0; 
        public static final double kTargetAngleAmp = 98; // 98; // 95; // 98 // 102; //32; // 35.6 // 109.0
        public static final double kTargetAngleCrossfield = 13.0;
        public static final double kTargetAnglePodium = 32.0; // TODO:tune
        public static final double kTargetAngleSpeakerFromAmpZone = kTargetAngleStowed; // TODO: tune
    }

    public static final class Shooter {
        public static final double kSupplyCurrentLimitShooter = 0.0;
        public static final double kSupplyCurrentLimitIndexer = 0.0;     

        public static final boolean kIsShooterTopInverted = false;
        public static final boolean kIsShooterBottomInverted = false;
        public static final boolean kIsIndexerInverted = false;

        public static final double kShooterSpeedScoreSpeakerSubwoofer = 18.0; // 17.0
        public static final double kShooterSpeedScoreAmp = 0.3; // -0.25
        public static final double kShooterEjectNoteSpeed = -0.25;
        public static final double kShooterIntakeNoteSpeed = 0.0;
        public static final double kShooterSpeedRollforward = 0.25;
        public static final double kShooterSpeedRollbackPercent = -0.1;
        public static final double kShooterSpeedCrossField = 22.0; 
        public static final double kShooterSpeedScoreSpeakerPodium = 22.0;
        public static final double kShooterSpeedScoreSpeakerAmpZone = kShooterSpeedScoreSpeakerSubwoofer;

        public static final double kIndexerSpeedIntake = 4.5; // 0.2 
        public static final double kIndexerSpeedScoreSpeaker = 6.5; // 0.9
        public static final double kIndexerSpeedScoreAmp = 0.4; // -0.25
        public static final double kIndexerEjectNoteSpeed = -0.25;
        public static final double kIndexerSpeedRollforward = 3.0;
        public static final double kIndexerSpeedRollback = -2.5;
        public static final double kIndexerSpeedCrossField = 6.5;

        public static final double kShooterBottomSpeedMultiplier = 0.98;

        public static final double kManualIntakeSupplierTolerance = 0.05;
        public static final double kManualSpeedMultiplier = 0.1;
        
        public static final double kShooterkIndexerIncramentMultiplier = 0.01;
        public static final double kShooterIncramentMultiplier = 0.01;

        public static final double kShooterGearRatio = 0.6666666666;
        public static final double kIndexerGearRatio = 0.5;

        public static final double kShooterWheelDiameter = Conversions.inchesToMeters(2.375);
        public static final double kIndexerWheelDiameter = Conversions.inchesToMeters(2.0);

        public static final double kHasNoteCurrentThresholdIndexer = 75.0; // 65.0, 90.0
        public static final double kHasNoteCurrentThresholdShooter = 15.0;
        public static final double kIntakeRollbackCurrentThresholdIndexer = 25.0;
        public static final double kIntakeRollbackCurrentThresholdShooter = 10.0;

        public static final double kMaxIndexerOutput = 0.9;

        public static final double kAtGoalTolerance = 0.25; // 0.05 = precise for dynamic

        public static double kPShooter = 4.5;
        public static double kIShooter = 0.0;
        public static double kDShooter = 0.2;

        public static final double kPIndexer = 0.4;
        public static final double kIIndexer = 0.0;
        public static final double kDIndexer = 0.0;
    }

    public static final class Intake {
        public static final double kSupplyCurrentLimitPivot = 0.0;
        public static final double kSupplyCurrentLimitRoll = 0.0;

        public static final double kPivotAngleStowed = 23.0; // 20
        public static final double kPivotAngleIntake = 130.0;
        public static final double kPivotAngleEject = 30.0; // 55.0
        public static final double kPivotAngleScoreSpeakerSubwoofer = kPivotAngleStowed;
        public static final double kPivotAngleScoreSpeaker = 28.0; // 26.0;
        public static final double kIndexingAngleIntake = 20.0;  // 10.0
        public static final double kPivotAngleShootCrossfield = 30.0; // TODO: tune this
        public static final double kPivotAngleScoreAmp = 27.0;

        public static final double kRollSpeedStowed = 0.0;
        public static final double kRollSpeedIntake = 0.4;
        public static final double kRollSpeedIntakeRollback = 0.05;
        public static final double kRollSpeedScoreSpeaker = 0.3;
        public static final double kEjectNoteSpeed = -0.6;
        

        public static final boolean kIsRollInverted = true;
        public static final boolean kIsPivotInverted = false;

        // public static final double kPivotMaxVelocity = 0.0;
        // public static final double kPivotMaxAccel = 0.0;
        public static final double kMaxPivotOutput = 0.18;

        public static final double kAtGoalTolerance = 0.5; 
        public static final double kFFtoPIDPivotTransitionTolerance = 20.0;

        public static final double kPivotIntakeGearRatio = 36.0;

        public static final double kPPivot = 0.0058305929; // 1, 0.1
        public static final double kIPivot = 0.0;
        public static final double kDPivot = 0.0000748355;

        public static final double kFFPivot = 0.15;

    }

    public static final class Vision {

        // measurements
        public static final Translation3d targetPointBlue = new Translation3d(0.0, 5.648, 0); // 5.548  // Moved towards right red alliance 0.1m
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
        
        /**
         * @param dist Distance from bot shooter to targetPoint in meters
         * @return Array of 2 values: shooterPivot angle in degrees, shooter velocity in MPS
         */
        // public static double[] getValues(double dist) {
        //     int index = (int) ((dist - 0.5) * 100);
        //     double[] retArr = {vals[index][1], vals[index][2]};
        //     return retArr;
        // }

        

        // VALS HERE
        // distance in meters of april tag, angle from 0-90 (0 is horiz), shooter mps

    }

    public static final class Drivetrain {

        public static final double kSupplyCurrentLimitDrive = 0.0;
        public static final double kSupplyCurrentLimitTurn = 0.0;

        // Measurements
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kModuleOffsetMeters = 0.572;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kModuleOffsetMeters / 2, kModuleOffsetMeters / 2), // -, -
                new Translation2d(-kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2), // -, +
                new Translation2d(kModuleOffsetMeters / 2, kModuleOffsetMeters / 2), // +, -
                new Translation2d(kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2)); // +, +


        public static final double kHeadingLockPIDMaxOutput = 2.0;
        public static final double kHeadingLockDegreeRejectionTolerance = 10.0;
        public static final double kHeadingLockDegreeTolerance = 1.0;


        // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        //         new Translation2d(-kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
        //         new Translation2d(-kModuleOffsetMeters / 2, kModuleOffsetMeters / 2),
        //         new Translation2d(kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
        //         new Translation2d(kModuleOffsetMeters / 2, kModuleOffsetMeters / 2));

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

        public static final double kPHeading = 1.6; // 1.2 // 0.6
        public static final double kIHeading = 0.0000;
        public static final double kDHeading = 0.0;

        public static PIDConst kPIDSpeakerHeadingLock = new PIDConst(1.0, 0, 0); // TODO: tune PID note heading lock is in radians while heading controller is in degrees

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

        public static double kMaxSpeed = 0.8; // 0.8 //1.0 // 0.6 - latest
        public static double kMaxAccel = 2.3; // 1.0 // 1.9 


        public static double kMaxTurnSpeed = 0.95; // 0.2 //1.1 // 0.08 WITH PID, 1.35 WITHOUT
        public static double kMaxTurnAccel = 10.0; // 1.5; // Instant manual turning

        

        // Misc
        public static final double kDrivingMotorDeadband = 0.05;

        public static final double kIntakeNoteSpeed = -0.3;

        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(kPTranslationPP, kITranslationPP, kDTranslationPP),
                new PIDConstants(kPRotationPP, kIRotationPP, kDRotationPP),
                kMaxDriveModuleSpeedMPS,
                kModuleOffsetMeters,
                new ReplanningConfig());

        public static final boolean kIsFieldOriented = true;

        public static boolean isPrecise = false;
        public static final double kPreciseMultiplier = 0.25;

        public static final double kTurnCurrentLimit = 100;
        public static final double kDriveCurrentLimit = 100;

        // public static boolean CAN_DIRECTION_SWITCH = false;

        public static final boolean FRONT_LEFT_D_IS_REVERSED = true;
        public static final boolean FRONT_RIGHT_D_IS_REVERSED = true;
        public static final boolean BACK_LEFT_D_IS_REVERSED = true;
        public static final boolean BACK_RIGHT_D_IS_REVERSED = true;

        public static final boolean FRONT_LEFT_T_IS_REVERSED = true;
        public static final boolean FRONT_RIGHT_T_IS_REVERSED = true;
        public static final boolean BACK_LEFT_T_IS_REVERSED = true;
        public static final boolean BACK_RIGHT_T_IS_REVERSED = true;
        

        // public static double kCANCoderOffsetFrontLeft = 207.9; // -53.7
        // public static double kCANCoderOffsetFrontRight = 307.0; // -331.4
        // public static double kCANCoderOffsetBackLeft = 80.2; // 131.1
        // public static double kCANCoderOffsetBackRight = 344.3; // 234.1
        // public static double kCANCoderOffsetFrontLeft = 24.1; // -53.7
        // public static double kCANCoderOffsetFrontRight = 315.2; // -331.4
        // public static double kCANCoderOffsetBackLeft = 260.5; // 131.1
        // public static double kCANCoderOffsetBackRight = 340.9; // 234.1
        
        // in rotations
        public static double kCANCoderOffsetFrontLeft = -0.055908; // 236.1;
        public static double kCANCoderOffsetFrontRight = -0.098389; // 302.0
        public static double kCANCoderOffsetBackLeft = -0.357178; // 113.4
        public static double kCANCoderOffsetBackRight = -0.013916; // 338.7


        // public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(303.2);
        // public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(305.5);
        // public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(44.6);
        // public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(77.5);

    }




    public static final class Controller {
        public static final double kHasNoteRumbleTimeSec = 0.5;
        public static final double kHasNoteRumbleIntensity = 0.8;
        
        public static final double kClimbTriggerAxisPercent = 0.1;
    }

}
