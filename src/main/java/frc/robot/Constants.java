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
        public static final double kIndexerIntakeRollBackTimeSec = 0.0; // 0.2

        // Elapsed time all elements at wait speaker pos before score speaker
        public static final double kWaitSpeakerTimeToleranceSec = 0.1;

        // Elapsed time shooting into speaker before stow
        public static final double kScoreSpeakerShootTimeToleranceSec = 1.5; // intake fallback = 0.5s,\1.0

        public static final double kScoreSpeakerPivotTimeToleranceSec = 0.3;

        //
        public static final double kScoreSpeakerHeadingTolerance = 0.25;
    }

    public static final class Elevator {
        public static final double kElevatorHeightStowed = 0.0;
        public static final double kElevatorHeightSpeaker = 0.0;
        public static final double kElevatorHeightSpeakerHigh = 0.0;
        public static final double kElevatorHeightAmp = 0.0;
        public static final double kElevatorHeightTrap = 0.0; 

        public static final PIDConst kPIDElevator = new PIDConst(0.0, 0.0, 0.0);

        public static final double kManualMultiplier = 0.1;

        public static final double kAtGoalTolerance = 0.005;

        public static final double kElevatorGearRatio = 18.89;
        public static final double kElevatorGearCircumM = Conversions.inchesToMeters(0.0);

        public static final boolean kIsInvertedLeft = false;
        public static final boolean kIsInvertedRight = true;

        public static final double kMaxVelocity = 0.0;
        public static final double kMaxAccel = 0.0;
    }

    public static final class ShooterPivot {
        public static final boolean kIsShooterPivotInverted = false;

        public static PIDConst kPID = new PIDConst(0.00675, 0, 0, 0.015); // ff 0.015

        public static final double kManualMultiplier = 0.001;

        public static final double kAtGoalTolerance = 0.25;

        public static final double kShooterPivotGearRatio = 148.15;

        public static final double kHorizontalAngle = 52.0;

        public static final double kMaxShooterPivotOutput = 1;

        public static final double kMaxDistance = 6.0;

        // public static final double kMaxVelocityDeg = 0.0;
        // public static final double kMaxAccelerationDeg = 0.0;

        public static final double kTargetAngleStowed = 5.0;
        public static final double kTargetAngleIntake = 5.0;
        public static final double kTargetAngleEject = 60.0;
        // public static final double kTargetAngleSpeaker = 0.0;
        // public static final double kTargetAngleSpeakerHigh = 0.0;
        public static final double kTargetAngleSpeakerFromAmp = kTargetAngleStowed;
        public static final double kTargetAngleSpeakerFromPodium = kTargetAngleStowed;
        public static final double kTargetAngleSpeakerFromSubwoofer = 10.0;
        public static final double kTargetAngleAmp = kTargetAngleStowed;
        public static final double kTargetAngleSource = kTargetAngleStowed;
    }

    public static final class Shooter {
        public static final boolean kIsShooterTopInverted = false;
        public static final boolean kIsShooterBottomInverted = false;
        public static final boolean kIsIndexerInverted = false;

        public static final double kShooterSpeedScoreSpeakerSubwoofer = 15.0; // 10.0
        public static final double kShooterSpeedScoreSpeakerPodium = 5.0;
        public static final double kShooterSpeedScoreSpeakerAmpZone = 10.0;
        public static final double kShooterSpeedScoreAmp = 1.0;
        public static final double kShooterEjectNoteSpeed = -0.25;
        public static final double kShooterIntakeNoteSpeed = 0.3;

        public static final double kIndexerSpeedIntake = 3; // 0.2
        public static final double kIndexerSpeedScoreSpeaker = 4.0; // 0.9
        public static final double kIndexerSpeedScoreAmp = 0.4;
        public static final double kIndexerEjectNoteSpeed = -0.5;
        public static final double kIndexerRollbackSpeed = -0.1;


        public static final double kManualIntakeSupplierTolerance = 0.05;
        public static final double kManualSpeedMultiplier = 0.1;
        
        // JUST IN CASE
        public static final double kIndexerSpeedSource = -0.1;

        public static final double kShooterkIndexerIncramentMultiplier = 0.01;
        public static final double kShooterIncramentMultiplier = 0.01;

        public static final double kShooterGearRatio = 0.6666666666;
        public static final double kIndexerGearRatio = 0.5;

        public static final double kShooterWheelDiameter = Conversions.inchesToMeters(2.375);
        public static final double kIndexerWheelDiameter = Conversions.inchesToMeters(2.0);

        public static final double kHasNoteCurrentThreshold = 90.0; // 65.0, 90.0
        public static final double kMaxIndexerOutput = 0.9;

        public static final double kAtGoalTolerance = 0.1; // 0.05 = precise for dynamic

        public static double kPShooter = 0.4;
        public static double kIShooter = 0.0;
        public static double kDShooter = 0.0075;

        public static final double kPIndexer = 0.4;
        public static final double kIIndexer = 0.0;
        public static final double kDIndexer = 0.0;
    }

    public static final class Intake {
        public static final double kPivotAngleStowed = 20.0; // 20
        public static final double kPivotAngleIntake = 120.0;
        public static final double kScoreSpeaker = 80.0;

        public static final double kRollSpeedStowed = 0.0;
        public static final double kRollSpeedIntake = 0.4;
        public static final double kRollSpeedIntakeRollback = 0.05;
        public static final double kRollSpeedScoreSpeaker = 0.2;
        public static final double kEjectNoteSpeed = -0.6;
        

        public static final boolean kIsRollInverted = true;
        public static final boolean kIsPivotInverted = false;

        // public static final double kPivotMaxVelocity = 0.0;
        // public static final double kPivotMaxAccel = 0.0;
        public static final double kMaxPivotOutput = 0.15;

        public static final double kAtGoalTolerance = 5; // 0.5

        public static final double kPivotIntakeGearRatio = 36.0;

        public static final double kPPivot = 0.002914; // 1, 0.1
        public static final double kIPivot = 0.0;
        public static final double kDPivot = 0.00002445;

    }

    public static final class Vision {

        // measurements
        public static final double kSpeakerAprilTagHeightMeters = 1.451;
        public static final double kLimelightPitchAngleDegrees = 5.0;
        public static final double kLimelightHeightMeters = 0.534;

        public static final double kAcceptableVolatilityThreshold = 0.2;

        public static final double kMaxDrivingSpeed = 0.0;
        public static final double kMaxTurningSpeed = 0.1;

        public static final int kVolatilitySlidingWindowLen = 20;

        public static final double kdistanceOffset = 0.7642; // 0.5992
    }

    public static final class Drivetrain {
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

        // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        //         new Translation2d(-kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
        //         new Translation2d(-kModuleOffsetMeters / 2, kModuleOffsetMeters / 2),
        //         new Translation2d(kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
        //         new Translation2d(kModuleOffsetMeters / 2, kModuleOffsetMeters / 2));

        // PID
        public static double kPTurnMotorFL = 0.00255; // 0.0125
        public static double kITurnMotorFL = 0.00;
        public static double kDTurnMotorFL = 0.000025; // 0.000026

        public static double kPTurnMotorFR = 0.0027; // 0.0125
        public static double kITurnMotorFR = 0.00;
        public static double kDTurnMotorFR = 0.0000; // 0.000026

        public static double kPTurnMotorBL = 0.00255; // 0.0125
        public static double kITurnMotorBL = 0.00;
        public static double kDTurnMotorBL = 0.000025; // 0.000026

        public static double kPTurnMotorBR = 0.00255; // 0.0125
        public static double kITurnMotorBR = 0.00;
        public static double kDTurnMotorBR = 0.000025; // 0.000026

        public static final double kPHeading = 1.2; // 0.6
        public static final double kIHeading = 0.0000;
        public static final double kDHeading = 0.0;

        public static final PIDConst kPIDSpeakerHeadingLock = new PIDConst(0.005, 0, 0);

        public static final double kPTranslationPP = 3.0; // 5.5
        public static final double kITranslationPP = 0.0;
        public static final double kDTranslationPP = 0.0;

        public static final double kPRotationPP = 3.0; // 3.0
        public static final double kIRotationPP = 0.0;
        public static final double kDRotationPP = 0.0; // 0.0


        public static final double kTranslationMultPP = 0.2;


        // Speed/Accel
        public static final double kMaxDriveModuleSpeedMPS = 4.0;

        public static double kMaxSpeed = 1.1; // 0.8 //1.0 // 0.6 - latest
        public static double kMaxAccel = 2.5; // 1.0 // 1.9 


        public static double kMaxTurnSpeed = 1.35; // 0.08 WITH PID, 1.35 WITHOUT
        public static double kMaxTurnAccel = 10; // Instant manual turning

        

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
        public static double kCANCoderOffsetFrontLeft = -0.062988; // 236.1;
        public static double kCANCoderOffsetFrontRight = -0.092041; // 302.0
        public static double kCANCoderOffsetBackLeft = -0.457520; // 113.4
        public static double kCANCoderOffsetBackRight = -0.017578; // 338.7

        // public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(303.2);
        // public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(305.5);
        // public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(44.6);
        // public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD =
        // Math.toRadians(77.5);

    }

    public static final class Auton {
        public static final boolean kFacingDriversOnStart = true;

        // Balance
        public static final double kChargeStationAngle = 14.0; // TODO: tune (pitch angle of bot on charge station)
        public static final double kOnChargeStationTolerance = 1.0; // TODO: tune (pitch angle tolerance for bot to be
                                                                    // considered on charge station)
        public static final double kChargeStationBalanceTolerance = 1.0; // TODO: tune (pitch angle tolerance while
                                                                         // climbing for charge station to be considered
                                                                         // falling)
        public static final double BALANCE_END_TIME_THRESHOLD = 0.5; // 0.5
        public static final double BALANCE_END_ANGLE_THRESHOLD = 5;
        public static final double kSpeedWhileClimbing = 0.02;

        // PID Turn
        public static final double kPTurn = 0.3; // 0.2
        public static final double kITurn = 0;
        public static final double kDTurn = 0.0015; // 0.001

        // Auton Parser
        // public static final double[][] ROUTINE = {{0.0, 0.0, 0.0, 0.0,
        // 2.356194490192345, -2.356194490192345, 0.7853981633974483,
        // -0.7853981633974483}, {0.0, 0.0, 0.0, 0.0, 2.356194490192345,
        // -2.356194490192345, 0.7853981633974483, -0.7853981633974483},
        // {0.01632753567810988, 0.01632753567810988, 0.01632753567810988,
        // 0.01632753567810988, 0.5992400782190582, 0.5992400782190582,
        // 0.5992400782190582, 0.5992400782190582}, {0.045247694500382775,
        // 0.045247694500382775, 0.045247694500382775, 0.045247694500382775,
        // 0.6846573169729425, 0.6846573169729425, 0.6846573169729425,
        // 0.6846573169729425}, {0.06266177140471169, 0.06266177140471169,
        // 0.06266177140471169, 0.06266177140471169, 0.5091910067230891,
        // 0.5091910067230891, 0.5091910067230891, 0.5091910067230891},
        // {0.08016732334164174, 0.08016732334164174, 0.08016732334164174,
        // 0.08016732334164174, 0.32434342321250337, 0.32434342321250337,
        // 0.32434342321250337, 0.32434342321250337}, {0.091932882326037,
        // 0.091932882326037, 0.091932882326037, 0.091932882326037, 0.10671194840788394,
        // 0.10671194840788394, 0.10671194840788394, 0.10671194840788394},
        // {0.11471118539925268, 0.11471118539925268, 0.11471118539925268,
        // 0.11471118539925268, -0.12221485846378785, -0.12221485846378785,
        // -0.12221485846378785, -0.12221485846378785}, {0.13380261262755627,
        // 0.13380261262755627, 0.13380261262755627, 0.13380261262755627,
        // -0.1101202332569859, -0.1101202332569859, -0.1101202332569859,
        // -0.1101202332569859}, {0.15311012579147015, 0.15311012579147015,
        // 0.15311012579147015, 0.15311012579147015, 0.02857223334470323,
        // 0.02857223334470323, 0.02857223334470323, 0.02857223334470323},
        // {0.17262679999999905, 0.17262679999999905, 0.17262679999999905,
        // 0.17262679999999905, -0.026589199911371875, -0.026589199911371875,
        // -0.026589199911371875, -0.026589199911371875}, {0.1696616057362413,
        // 0.1696616057362413, 0.1696616057362413, 0.1696616057362413,
        // 0.09192166589838945, 0.09192166589838945, 0.09192166589838945,
        // 0.09192166589838945}, {0.14946914366316486, 0.14946914366316486,
        // 0.14946914366316486, 0.14946914366316486, 0.07649652311010587,
        // 0.07649652311010587, 0.07649652311010587, 0.07649652311010587},
        // {0.12854159999999834, 0.12854159999999834, 0.12854159999999834,
        // 0.12854159999999834, -0.017950274699994665, -0.017950274699994665,
        // -0.017950274699994665, -0.017950274699994665}, {0.1086057999999994,
        // 0.1086057999999994, 0.1086057999999994, 0.1086057999999994,
        // -0.01494161707287134, -0.01494161707287134, -0.01494161707287134,
        // -0.01494161707287134}, {0.08996351522967358, 0.08996351522967358,
        // 0.08996351522967358, 0.08996351522967358, -0.1946322226946833,
        // -0.1946322226946833, -0.1946322226946833, -0.1946322226946833},
        // {0.07725821375413663, 0.07725821375413663, 0.07725821375413663,
        // 0.07725821375413663, -0.49731800029766904, -0.49731800029766904,
        // -0.49731800029766904, -0.49731800029766904}, {0.05093995690025627,
        // 0.05093995690025627, 0.05093995690025627, 0.05093995690025627,
        // -0.32790321829266794, -0.32790321829266794, -0.32790321829266794,
        // -0.32790321829266794}, {0.028568599999998417, 0.028568599999998417,
        // 0.028568599999998417, 0.028568599999998417, -0.010528990596116563,
        // -0.010528990596116563, -0.010528990596116563, -0.010528990596116563},
        // {0.03446327558488935, 0.03446327558488935, 0.03446327558488935,
        // 0.03446327558488935, 0.35335570156577917, 0.35335570156577917,
        // 0.35335570156577917, 0.35335570156577917}, {0.039735920082465286,
        // 0.039735920082465286, 0.039735920082465286, 0.039735920082465286,
        // 0.41649042051391627, 0.41649042051391627, 0.41649042051391627,
        // 0.41649042051391627}, {0.016072400000000188, 0.016072400000000188,
        // 0.016072400000000188, 0.016072400000000188, 0.007900858168476077,
        // 0.007900858168476077, 0.007900858168476077, 0.007900858168476077}, {0.0, 0.0,
        // 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462,
        // 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462,
        // 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0,
        // 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462,
        // 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462,
        // 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0,
        // 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462,
        // 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462,
        // 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0,
        // 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462,
        // 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462,
        // 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0,
        // 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462,
        // 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462,
        // 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0,
        // 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462,
        // 0.007293827738426462}};

        // PID Drive
        // public static final double kPDriveToTargetXY = 1;
        // public static final double kIDriveToTargetXY = 0.0;
        // public static final double kDDriveToTargetXY = 0.0;

        // public static final double kPDriveToTargetT = 1;
        // public static final double kIDriveToTargetT = 0.0;
        // public static final double kDDriveToTargetT = 0.0;
    }

    public static final class ShootingSetpoints {
        // public static final Translation3d targetPointBlue = new Translation3d(-0.1,
        // 5.548, 0);
        // public static final Translation3d targetPointRed = new Translation3d(-0.1,
        // (8.211-5.548), 0);

        // Most likely can be same blue/red
        public static final Translation3d targetPoint = new Translation3d(-0.1, (8.211 - 5.548), 0);


        
        /**
         * @param dist Distance from bot shooter to targetPoint in meters
         * @return Array of 2 values: shooterPivot angle in degrees, shooter velocity in MPS
         */
        public static double[] getValues(double dist) {
            int index = (int) ((dist - 0.5) * 100);
            double[] retArr = {vals[index][1], vals[index][2]};
            return retArr;
        }

        // Values below, nothing below vals
        public static final double[][] vals = {{0.5, 70.92, 10.75}, {0.51, 70.57, 10.765}, {0.52, 70.22, 10.78}, {0.53, 69.87, 10.795}, {0.54, 69.52, 10.81}, {0.55, 69.18, 10.825}, {0.56, 68.84, 10.84}, {0.57, 68.5, 10.855}, {0.58, 68.16, 10.87}, {0.59, 67.82, 10.885}, {0.6, 67.49, 10.9}, {0.61, 67.15, 10.915}, {0.62, 66.82, 10.93}, {0.63, 66.49, 10.945}, {0.64, 66.16, 10.96}, {0.65, 65.84, 10.975}, {0.66, 65.51, 10.99}, {0.67, 65.19, 11.005}, {0.68, 64.87, 11.02}, {0.69, 64.55, 11.035}, {0.7, 64.24, 11.05}, {0.71, 63.92, 11.065}, {0.72, 63.61, 11.08}, {0.73, 63.3, 11.095}, {0.74, 62.99, 11.11}, {0.75, 62.68, 11.125}, {0.76, 62.38, 11.14}, {0.77, 62.07, 11.155}, {0.78, 61.77, 11.17}, {0.79, 61.47, 11.185}, {0.8, 61.18, 11.2}, {0.81, 60.88, 11.215}, {0.82, 60.59, 11.23}, {0.83, 60.3, 11.245}, {0.84, 60.01, 11.26}, {0.85, 59.72, 11.275}, {0.86, 59.44, 11.29}, {0.87, 59.15, 11.305}, {0.88, 58.87, 11.32}, {0.89, 58.59, 11.335}, {0.9, 58.31, 11.35}, {0.91, 58.04, 11.365}, {0.92, 57.76, 11.38}, {0.93, 57.49, 11.395}, {0.94, 57.22, 11.41}, {0.95, 56.95, 11.425}, {0.96, 56.69, 11.44}, {0.97, 56.42, 11.455}, {0.98, 56.16, 11.47}, {0.99, 55.9, 11.485}, {1.0, 55.64, 11.5}, {1.01, 55.38, 11.515}, {1.02, 55.13, 11.53}, {1.03, 54.87, 11.545}, {1.04, 54.62, 11.56}, {1.05, 54.37, 11.575}, {1.06, 54.12, 11.59}, {1.07, 53.88, 11.605}, {1.08, 53.63, 11.62}, {1.09, 53.39, 11.635}, {1.1, 53.15, 11.65}, {1.11, 52.91, 11.665}, {1.12, 52.67, 11.68}, {1.13, 52.44, 11.695}, {1.14, 52.2, 11.71}, {1.15, 51.97, 11.725}, {1.16, 51.74, 11.74}, {1.17, 51.51, 11.755}, {1.18, 51.29, 11.77}, {1.19, 51.06, 11.785}, {1.2, 50.84, 11.8}, {1.21, 50.62, 11.815}, {1.22, 50.4, 11.83}, {1.23, 50.18, 11.845}, {1.24, 49.96, 11.86}, {1.25, 49.75, 11.875}, {1.26, 49.53, 11.89}, {1.27, 49.32, 11.905}, {1.28, 49.11, 11.92}, {1.29, 48.9, 11.935}, {1.3, 48.69, 11.95}, {1.31, 48.49, 11.965}, {1.32, 48.28, 11.98}, {1.33, 48.08, 11.995}, {1.34, 47.88, 12.01}, {1.35, 47.68, 12.025}, {1.36, 47.48, 12.04}, {1.37, 47.28, 12.055}, {1.38, 47.09, 12.07}, {1.39, 46.89, 12.085}, {1.4, 46.7, 12.1}, {1.41, 46.51, 12.115}, {1.42, 46.32, 12.13}, {1.43, 46.13, 12.145}, {1.44, 45.95, 12.16}, {1.45, 45.76, 12.175}, {1.46, 45.58, 12.19}, {1.47, 45.4, 12.205}, {1.48, 45.21, 12.22}, {1.49, 45.04, 12.235}, {1.5, 44.86, 12.25}, {1.51, 44.68, 12.265}, {1.52, 44.5, 12.28}, {1.53, 44.33, 12.295}, {1.54, 44.16, 12.31}, {1.55, 43.98, 12.325}, {1.56, 43.81, 12.34}, {1.57, 43.64, 12.355}, {1.58, 43.48, 12.37}, {1.59, 43.31, 12.385}, {1.6, 43.14, 12.4}, {1.61, 42.98, 12.415}, {1.62, 42.82, 12.43}, {1.63, 42.65, 12.445}, {1.64, 42.49, 12.46}, {1.65, 42.33, 12.475}, {1.66, 42.18, 12.49}, {1.67, 42.02, 12.505}, {1.68, 41.86, 12.52}, {1.69, 41.71, 12.535}, {1.7, 41.55, 12.55}, {1.71, 41.4, 12.565}, {1.72, 41.25, 12.58}, {1.73, 41.1, 12.595}, {1.74, 40.95, 12.61}, {1.75, 40.8, 12.625}, {1.76, 40.65, 12.64}, {1.77, 40.51, 12.655}, {1.78, 40.36, 12.67}, {1.79, 40.22, 12.685}, {1.8, 40.08, 12.7}, {1.81, 39.93, 12.715}, {1.82, 39.79, 12.73}, {1.83, 39.65, 12.745}, {1.84, 39.51, 12.76}, {1.85, 39.38, 12.775}, {1.86, 39.24, 12.79}, {1.87, 39.1, 12.805}, {1.88, 38.97, 12.82}, {1.89, 38.83, 12.835}, {1.9, 38.7, 12.85}, {1.91, 38.57, 12.865}, {1.92, 38.44, 12.88}, {1.93, 38.31, 12.895}, {1.94, 38.18, 12.91}, {1.95, 38.05, 12.925}, {1.96, 37.92, 12.94}, {1.97, 37.79, 12.955}, {1.98, 37.67, 12.97}, {1.99, 37.54, 12.985}, {2.0, 37.42, 13.0}, {2.01, 37.3, 13.015}, {2.02, 37.17, 13.03}, {2.03, 37.05, 13.045}, {2.04, 36.93, 13.06}, {2.05, 36.81, 13.075}, {2.06, 36.69, 13.09}, {2.07, 36.57, 13.105}, {2.08, 36.46, 13.12}, {2.09, 36.34, 13.135}, {2.1, 36.22, 13.15}, {2.11, 36.11, 13.165}, {2.12, 36.0, 13.18}, {2.13, 35.88, 13.195}, {2.14, 35.77, 13.21}, {2.15, 35.66, 13.225}, {2.16, 35.55, 13.24}, {2.17, 35.44, 13.255}, {2.18, 35.33, 13.27}, {2.19, 35.22, 13.285}, {2.2, 35.11, 13.3}, {2.21, 35.0, 13.315}, {2.22, 34.89, 13.33}, {2.23, 34.79, 13.345}, {2.24, 34.68, 13.36}, {2.25, 34.58, 13.375}, {2.26, 34.47, 13.39}, {2.27, 34.37, 13.405}, {2.28, 34.27, 13.42}, {2.29, 34.17, 13.435}, {2.3, 34.06, 13.45}, {2.31, 33.96, 13.465}, {2.32, 33.86, 13.48}, {2.33, 33.76, 13.495}, {2.34, 33.67, 13.51}, {2.35, 33.57, 13.525}, {2.36, 33.47, 13.54}, {2.37, 33.37, 13.555}, {2.38, 33.28, 13.57}, {2.39, 33.18, 13.585}, {2.4, 33.09, 13.6}, {2.41, 32.99, 13.615}, {2.42, 32.9, 13.63}, {2.43, 32.81, 13.645}, {2.44, 32.71, 13.66}, {2.45, 32.62, 13.675}, {2.46, 32.53, 13.69}, {2.47, 32.44, 13.705}, {2.48, 32.35, 13.72}, {2.49, 32.26, 13.735}, {2.5, 32.17, 13.75}, {2.51, 32.08, 13.765}, {2.52, 31.99, 13.78}, {2.53, 31.9, 13.795}, {2.54, 31.82, 13.81}, {2.55, 31.73, 13.825}, {2.56, 31.64, 13.84}, {2.57, 31.56, 13.855}, {2.58, 31.47, 13.87}, {2.59, 31.39, 13.885}, {2.6, 31.31, 13.9}, {2.61, 31.22, 13.915}, {2.62, 31.14, 13.93}, {2.63, 31.06, 13.945}, {2.64, 30.98, 13.96}, {2.65, 30.89, 13.975}, {2.66, 30.81, 13.99}, {2.67, 30.73, 14.005}, {2.68, 30.65, 14.02}, {2.69, 30.57, 14.035}, {2.7, 30.49, 14.05}, {2.71, 30.42, 14.065}, {2.72, 30.34, 14.08}, {2.73, 30.26, 14.095}, {2.74, 30.18, 14.11}, {2.75, 30.11, 14.125}, {2.76, 30.03, 14.14}, {2.77, 29.95, 14.155}, {2.78, 29.88, 14.17}, {2.79, 29.8, 14.185}, {2.8, 29.73, 14.2}, {2.81, 29.66, 14.215}, {2.82, 29.58, 14.23}, {2.83, 29.51, 14.245}, {2.84, 29.44, 14.26}, {2.85, 29.36, 14.275}, {2.86, 29.29, 14.29}, {2.87, 29.22, 14.305}, {2.88, 29.15, 14.32}, {2.89, 29.08, 14.335}, {2.9, 29.01, 14.35}, {2.91, 28.94, 14.365}, {2.92, 28.87, 14.38}, {2.93, 28.8, 14.395}, {2.94, 28.73, 14.41}, {2.95, 28.66, 14.425}, {2.96, 28.59, 14.44}, {2.97, 28.53, 14.455}, {2.98, 28.46, 14.47}, {2.99, 28.39, 14.485}, {3.0, 28.33, 14.5}, {3.01, 28.26, 14.515}, {3.02, 28.19, 14.53}, {3.03, 28.13, 14.545}, {3.04, 28.06, 14.56}, {3.05, 28.0, 14.575}, {3.06, 27.93, 14.59}, {3.07, 27.87, 14.605}, {3.08, 27.81, 14.62}, {3.09, 27.74, 14.635}, {3.1, 27.68, 14.65}, {3.11, 27.62, 14.665}, {3.12, 27.56, 14.68}, {3.13, 27.49, 14.695}, {3.14, 27.43, 14.71}, {3.15, 27.37, 14.725}, {3.16, 27.31, 14.74}, {3.17, 27.25, 14.755}, {3.18, 27.19, 14.77}, {3.19, 27.13, 14.785}, {3.2, 27.07, 14.8}, {3.21, 27.01, 14.815}, {3.22, 26.95, 14.83}, {3.23, 26.89, 14.845}, {3.24, 26.83, 14.86}, {3.25, 26.77, 14.875}, {3.26, 26.72, 14.89}, {3.27, 26.66, 14.905}, {3.28, 26.6, 14.92}, {3.29, 26.54, 14.935}, {3.3, 26.49, 14.95}, {3.31, 26.43, 14.965}, {3.32, 26.38, 14.98}, {3.33, 26.32, 14.995}, {3.34, 26.26, 15.01}, {3.35, 26.21, 15.025}, {3.36, 26.15, 15.04}, {3.37, 26.1, 15.055}, {3.38, 26.04, 15.07}, {3.39, 25.99, 15.085}, {3.4, 25.94, 15.1}, {3.41, 25.88, 15.115}, {3.42, 25.83, 15.13}, {3.43, 25.78, 15.145}, {3.44, 25.72, 15.16}, {3.45, 25.67, 15.175}, {3.46, 25.62, 15.19}, {3.47, 25.57, 15.205}, {3.48, 25.51, 15.22}, {3.49, 25.46, 15.235}, {3.5, 25.41, 15.25}, {3.51, 25.36, 15.265}, {3.52, 25.31, 15.28}, {3.53, 25.26, 15.295}, {3.54, 25.21, 15.31}, {3.55, 25.16, 15.325}, {3.56, 25.11, 15.34}, {3.57, 25.06, 15.355}, {3.58, 25.01, 15.37}, {3.59, 24.96, 15.385}, {3.6, 24.91, 15.4}, {3.61, 24.86, 15.415}, {3.62, 24.81, 15.43}, {3.63, 24.77, 15.445}, {3.64, 24.72, 15.46}, {3.65, 24.67, 15.475}, {3.66, 24.62, 15.49}, {3.67, 24.58, 15.505}, {3.68, 24.53, 15.52}, {3.69, 24.48, 15.535}, {3.7, 24.44, 15.55}, {3.71, 24.39, 15.565}, {3.72, 24.34, 15.58}, {3.73, 24.3, 15.595}, {3.74, 24.25, 15.61}, {3.75, 24.21, 15.625}, {3.76, 24.16, 15.64}, {3.77, 24.11, 15.655}, {3.78, 24.07, 15.67}, {3.79, 24.02, 15.685}, {3.8, 23.98, 15.7}, {3.81, 23.94, 15.715}, {3.82, 23.89, 15.73}, {3.83, 23.85, 15.745}, {3.84, 23.8, 15.76}, {3.85, 23.76, 15.775}, {3.86, 23.72, 15.79}, {3.87, 23.67, 15.805}, {3.88, 23.63, 15.82}, {3.89, 23.59, 15.835}, {3.9, 23.55, 15.85}, {3.91, 23.5, 15.865}, {3.92, 23.46, 15.88}, {3.93, 23.42, 15.895}, {3.94, 23.38, 15.91}, {3.95, 23.34, 15.925}, {3.96, 23.29, 15.94}, {3.97, 23.25, 15.955}, {3.98, 23.21, 15.97}, {3.99, 23.17, 15.985}, {4.0, 23.13, 16.0}, {4.01, 23.09, 16.015}, {4.02, 23.05, 16.03}, {4.03, 23.01, 16.045}, {4.04, 22.97, 16.06}, {4.05, 22.93, 16.075}, {4.06, 22.89, 16.09}, {4.07, 22.85, 16.105}, {4.08, 22.81, 16.12}, {4.09, 22.77, 16.135}, {4.1, 22.73, 16.15}, {4.11, 22.69, 16.165}, {4.12, 22.65, 16.18}, {4.13, 22.62, 16.195}, {4.14, 22.58, 16.21}, {4.15, 22.54, 16.225}, {4.16, 22.5, 16.24}, {4.17, 22.46, 16.255}, {4.18, 22.42, 16.27}, {4.19, 22.39, 16.285}, {4.2, 22.35, 16.3}, {4.21, 22.31, 16.315}, {4.22, 22.28, 16.33}, {4.23, 22.24, 16.345}, {4.24, 22.2, 16.36}, {4.25, 22.16, 16.375}, {4.26, 22.13, 16.39}, {4.27, 22.09, 16.405}, {4.28, 22.06, 16.42}, {4.29, 22.02, 16.435}, {4.3, 21.98, 16.45}, {4.31, 21.95, 16.465}, {4.32, 21.91, 16.48}, {4.33, 21.88, 16.495}, {4.34, 21.84, 16.51}, {4.35, 21.81, 16.525}, {4.36, 21.77, 16.54}, {4.37, 21.74, 16.555}, {4.38, 21.7, 16.57}, {4.39, 21.67, 16.585}, {4.4, 21.63, 16.6}, {4.41, 21.6, 16.615}, {4.42, 21.56, 16.63}, {4.43, 21.53, 16.645}, {4.44, 21.49, 16.66}, {4.45, 21.46, 16.675}, {4.46, 21.43, 16.69}, {4.47, 21.39, 16.705}, {4.48, 21.36, 16.72}, {4.49, 21.33, 16.735}, {4.5, 21.29, 16.75}, {4.51, 21.26, 16.765}, {4.52, 21.23, 16.78}, {4.53, 21.19, 16.795}, {4.54, 21.16, 16.81}, {4.55, 21.13, 16.825}, {4.56, 21.1, 16.84}, {4.57, 21.06, 16.855}, {4.58, 21.03, 16.87}, {4.59, 21.0, 16.885}, {4.6, 20.97, 16.9}, {4.61, 20.94, 16.915}, {4.62, 20.9, 16.93}, {4.63, 20.87, 16.945}, {4.64, 20.84, 16.96}, {4.65, 20.81, 16.975}, {4.66, 20.78, 16.99}, {4.67, 20.75, 17.005}, {4.68, 20.72, 17.02}, {4.69, 20.69, 17.035}, {4.7, 20.65, 17.05}, {4.71, 20.62, 17.065}, {4.72, 20.59, 17.08}, {4.73, 20.56, 17.095}, {4.74, 20.53, 17.11}, {4.75, 20.5, 17.125}, {4.76, 20.47, 17.14}, {4.77, 20.44, 17.155}, {4.78, 20.41, 17.17}, {4.79, 20.38, 17.185}, {4.8, 20.35, 17.2}, {4.81, 20.32, 17.215}, {4.82, 20.29, 17.23}, {4.83, 20.26, 17.245}, {4.84, 20.24, 17.26}, {4.85, 20.21, 17.275}, {4.86, 20.18, 17.29}, {4.87, 20.15, 17.305}, {4.88, 20.12, 17.32}, {4.89, 20.09, 17.335}, {4.9, 85.3, 17.35}, {4.91, 20.03, 17.365}, {4.92, 20.01, 17.38}, {4.93, 19.98, 17.395}, {4.94, 19.95, 17.41}, {4.95, 19.92, 17.425}, {4.96, 19.89, 17.44}, {4.97, 19.86, 17.455}, {4.98, 19.84, 17.47}, {4.99, 19.81, 17.485}, {5.0, 19.78, 17.5}, {5.01, 19.75, 17.515}, {5.02, 19.73, 17.53}, {5.03, 19.7, 17.545}, {5.04, 19.67, 17.56}, {5.05, 19.64, 17.575}, {5.06, 19.62, 17.59}, {5.07, 19.59, 17.605}, {5.08, 19.56, 17.62}, {5.09, 19.54, 17.635}, {5.1, 19.51, 17.65}, {5.11, 19.48, 17.665}, {5.12, 19.46, 17.68}, {5.13, 19.43, 17.695}, {5.14, 19.4, 17.71}, {5.15, 19.38, 17.725}, {5.16, 19.35, 17.74}, {5.17, 19.33, 17.755}, {5.18, 19.3, 17.77}, {5.19, 19.27, 17.785}, {5.2, 19.25, 17.8}, {5.21, 19.22, 17.815}, {5.22, 19.2, 17.83}, {5.23, 19.17, 17.845}, {5.24, 19.15, 17.86}, {5.25, 19.12, 17.875}, {5.26, 19.09, 17.89}, {5.27, 19.07, 17.905}, {5.28, 19.04, 17.92}, {5.29, 19.02, 17.935}, {5.3, 18.99, 17.95}, {5.31, 18.97, 17.965}, {5.32, 18.94, 17.98}, {5.33, 18.92, 17.995}, {5.34, 18.9, 18.01}, {5.35, 18.87, 18.025}, {5.36, 18.85, 18.04}, {5.37, 18.82, 18.055}, {5.38, 18.8, 18.07}, {5.39, 18.77, 18.085}, {5.4, 18.75, 18.1}, {5.41, 18.72, 18.115}, {5.42, 18.7, 18.13}, {5.43, 18.68, 18.145}, {5.44, 18.65, 18.16}, {5.45, 18.63, 18.175}, {5.46, 18.61, 18.19}, {5.47, 18.58, 18.205}, {5.48, 18.56, 18.22}, {5.49, 18.53, 18.235}, {5.5, 18.51, 18.25}, {5.51, 18.49, 18.265}, {5.52, 18.46, 18.28}, {5.53, 18.44, 18.295}, {5.54, 18.42, 18.31}, {5.55, 18.4, 18.325}, {5.56, 18.37, 18.34}, {5.57, 18.35, 18.355}, {5.58, 18.33, 18.37}, {5.59, 18.3, 18.385}, {5.6, 18.28, 18.4}, {5.61, 18.26, 18.415}, {5.62, 18.24, 18.43}, {5.63, 18.21, 18.445}, {5.64, 18.19, 18.46}, {5.65, 18.17, 18.475}, {5.66, 18.15, 18.49}, {5.67, 18.12, 18.505}, {5.68, 18.1, 18.52}, {5.69, 18.08, 18.535}, {5.7, 18.06, 18.55}, {5.71, 18.04, 18.565}, {5.72, 18.01, 18.58}, {5.73, 17.99, 18.595}, {5.74, 17.97, 18.61}, {5.75, 17.95, 18.625}, {5.76, 17.93, 18.64}, {5.77, 17.91, 18.655}, {5.78, 17.88, 18.67}, {5.79, 17.86, 18.685}, {5.8, 17.84, 18.7}, {5.81, 17.82, 18.715}, {5.82, 17.8, 18.73}, {5.83, 17.78, 18.745}, {5.84, 17.76, 18.76}, {5.85, 17.74, 18.775}, {5.86, 17.71, 18.79}, {5.87, 17.69, 18.805}, {5.88, 17.67, 18.82}, {5.89, 17.65, 18.835}, {5.9, 17.63, 18.85}, {5.91, 17.61, 18.865}, {5.92, 17.59, 18.88}, {5.93, 17.57, 18.895}, {5.94, 17.55, 18.91}, {5.95, 17.53, 18.925}, {5.96, 17.51, 18.94}, {5.97, 17.49, 18.955}, {5.98, 17.47, 18.97}, {5.99, 17.45, 18.985}, {6.0, 17.43, 19.0}, {6.01, 17.41, 19.015}, {6.02, 17.39, 19.03}, {6.03, 17.37, 19.045}, {6.04, 17.35, 19.06}, {6.05, 17.33, 19.075}, {6.06, 17.31, 19.09}, {6.07, 17.29, 19.105}, {6.08, 17.27, 19.12}, {6.09, 17.25, 19.135}, {6.1, 17.23, 19.15}, {6.11, 17.21, 19.165}, {6.12, 17.19, 19.18}, {6.13, 17.17, 19.195}, {6.14, 17.15, 19.21}, {6.15, 17.13, 19.225}, {6.16, 17.11, 19.24}, {6.17, 17.09, 19.255}, {6.18, 17.07, 19.27}, {6.19, 17.05, 19.285}, {6.2, 17.04, 19.3}, {6.21, 17.02, 19.315}, {6.22, 17.0, 19.33}, {6.23, 16.98, 19.345}, {6.24, 16.96, 19.36}, {6.25, 16.94, 19.375}, {6.26, 16.92, 19.39}, {6.27, 16.9, 19.405}, {6.28, 16.89, 19.42}, {6.29, 16.87, 19.435}, {6.3, 16.85, 19.45}, {6.31, 16.83, 19.465}, {6.32, 16.81, 19.48}, {6.33, 16.79, 19.495}, {6.34, 16.77, 19.51}, {6.35, 16.76, 19.525}, {6.36, 16.74, 19.54}, {6.37, 16.72, 19.555}, {6.38, 16.7, 19.57}, {6.39, 16.68, 19.585}, {6.4, 16.67, 19.6}, {6.41, 16.65, 19.615}, {6.42, 16.63, 19.63}, {6.43, 16.61, 19.645}, {6.44, 16.59, 19.66}, {6.45, 16.58, 19.675}, {6.46, 16.56, 19.69}, {6.47, 16.54, 19.705}, {6.48, 16.52, 19.72}, {6.49, 16.51, 19.735}, {6.5, 16.49, 19.75}, {6.51, 16.47, 19.765}, {6.52, 16.45, 19.78}, {6.53, 16.44, 19.795}, {6.54, 16.42, 19.81}, {6.55, 16.4, 19.825}, {6.56, 16.38, 19.84}, {6.57, 16.37, 19.855}, {6.58, 16.35, 19.87}, {6.59, 16.33, 19.885}, {6.6, 16.31, 19.9}, {6.61, 16.3, 19.915}, {6.62, 16.28, 19.93}, {6.63, 16.26, 19.945}, {6.64, 16.25, 19.96}, {6.65, 16.23, 19.975}, {6.66, 16.21, 19.99}, {6.67, 16.2, 20.005}, {6.68, 16.18, 20.02}, {6.69, 16.16, 20.035}, {6.7, 16.15, 20.05}, {6.71, 16.13, 20.065}, {6.72, 16.11, 20.08}, {6.73, 16.1, 20.095}, {6.74, 16.08, 20.11}, {6.75, 16.06, 20.125}, {6.76, 16.05, 20.14}, {6.77, 16.03, 20.155}, {6.78, 16.01, 20.17}, {6.79, 16.0, 20.185}, {6.8, 15.98, 20.2}, {6.81, 15.96, 20.215}, {6.82, 15.95, 20.23}, {6.83, 15.93, 20.245}, {6.84, 15.92, 20.26}, {6.85, 15.9, 20.275}, {6.86, 15.88, 20.29}, {6.87, 15.87, 20.305}, {6.88, 15.85, 20.32}, {6.89, 15.84, 20.335}, {6.9, 15.82, 20.35}, {6.91, 15.8, 20.365}, {6.92, 15.79, 20.38}, {6.93, 15.77, 20.395}, {6.94, 15.76, 20.41}, {6.95, 15.74, 20.425}, {6.96, 15.73, 20.44}, {6.97, 15.71, 20.455}, {6.98, 15.7, 20.47}, {6.99, 15.68, 20.485}, {7.0, 15.66, 20.5}, {7.01, 15.65, 20.515}, {7.02, 15.63, 20.53}, {7.03, 15.62, 20.545}, {7.04, 15.6, 20.56}, {7.05, 15.59, 20.575}, {7.06, 15.57, 20.59}, {7.07, 15.56, 20.605}, {7.08, 15.54, 20.62}, {7.09, 15.53, 20.635}, {7.1, 15.51, 20.65}, {7.11, 15.5, 20.665}, {7.12, 15.48, 20.68}, {7.13, 15.47, 20.695}, {7.14, 15.45, 20.71}, {7.15, 15.44, 20.725}, {7.16, 15.42, 20.74}, {7.17, 15.41, 20.755}, {7.18, 15.39, 20.77}, {7.19, 15.38, 20.785}, {7.2, 15.36, 20.8}, {7.21, 15.35, 20.815}, {7.22, 15.33, 20.83}, {7.23, 15.32, 20.845}, {7.24, 15.3, 20.86}, {7.25, 15.29, 20.875}, {7.26, 15.27, 20.89}, {7.27, 15.26, 20.905}, {7.28, 15.25, 20.92}, {7.29, 15.23, 20.935}, {7.3, 15.22, 20.95}, {7.31, 15.2, 20.965}, {7.32, 15.19, 20.98}, {7.33, 15.17, 20.995}, {7.34, 15.16, 21.01}, {7.35, 15.14, 21.025}, {7.36, 15.13, 21.04}, {7.37, 15.12, 21.055}, {7.38, 15.1, 21.07}, {7.39, 15.09, 21.085}, {7.4, 15.07, 21.1}, {7.41, 15.06, 21.115}, {7.42, 15.05, 21.13}, {7.43, 15.03, 21.145}, {7.44, 15.02, 21.16}, {7.45, 15.0, 21.175}, {7.46, 14.99, 21.19}, {7.47, 14.98, 21.205}, {7.48, 14.96, 21.22}, {7.49, 14.95, 21.235}, {7.5, 14.93, 21.25}, {7.51, 14.92, 21.265}, {7.52, 14.91, 21.28}, {7.53, 14.89, 21.295}, {7.54, 14.88, 21.31}, {7.55, 14.87, 21.325}, {7.56, 14.85, 21.34}, {7.57, 14.84, 21.355}, {7.58, 14.83, 21.37}, {7.59, 14.81, 21.385}, {7.6, 14.8, 21.4}, {7.61, 14.79, 21.415}, {7.62, 14.77, 21.43}, {7.63, 14.76, 21.445}, {7.64, 14.75, 21.46}, {7.65, 14.73, 21.475}, {7.66, 14.72, 21.49}, {7.67, 14.71, 21.505}, {7.68, 14.69, 21.52}, {7.69, 14.68, 21.535}, {7.7, 14.67, 21.55}, {7.71, 14.65, 21.565}, {7.72, 14.64, 21.58}, {7.73, 14.63, 21.595}, {7.74, 14.61, 21.61}, {7.75, 14.6, 21.625}, {7.76, 14.59, 21.64}, {7.77, 14.57, 21.655}, {7.78, 14.56, 21.67}, {7.79, 14.55, 21.685}, {7.8, 14.54, 21.7}, {7.81, 14.52, 21.715}, {7.82, 14.51, 21.73}, {7.83, 14.5, 21.745}, {7.84, 14.48, 21.76}, {7.85, 14.47, 21.775}, {7.86, 14.46, 21.79}, {7.87, 14.45, 21.805}, {7.88, 14.43, 21.82}, {7.89, 14.42, 21.835}, {7.9, 14.41, 21.85}, {7.91, 14.4, 21.865}, {7.92, 14.38, 21.88}, {7.93, 14.37, 21.895}, {7.94, 14.36, 21.91}, {7.95, 14.34, 21.925}, {7.96, 14.33, 21.94}, {7.97, 14.32, 21.955}, {7.98, 14.31, 21.97}, {7.99, 14.3, 21.985}, {8.0, 14.28, 22.0}, {8.01, 14.27, 22.015}, {8.02, 14.26, 22.03}, {8.03, 14.25, 22.045}, {8.04, 14.23, 22.06}, {8.05, 14.22, 22.075}, {8.06, 14.21, 22.09}, {8.07, 14.2, 22.105}, {8.08, 14.18, 22.12}, {8.09, 14.17, 22.135}, {8.1, 14.16, 22.15}, {8.11, 14.15, 22.165}, {8.12, 14.14, 22.18}, {8.13, 14.12, 22.195}, {8.14, 14.11, 22.21}, {8.15, 14.1, 22.225}, {8.16, 14.09, 22.24}, {8.17, 14.08, 22.255}, {8.18, 14.06, 22.27}, {8.19, 14.05, 22.285}, {8.2, 14.04, 22.3}, {8.21, 14.03, 22.315}, {8.22, 14.02, 22.33}, {8.23, 14.01, 22.345}, {8.24, 13.99, 22.36}, {8.25, 13.98, 22.375}, {8.26, 13.97, 22.39}, {8.27, 13.96, 22.405}, {8.28, 13.95, 22.42}, {8.29, 13.93, 22.435}, {8.3, 13.92, 22.45}, {8.31, 13.91, 22.465}, {8.32, 13.9, 22.48}, {8.33, 13.89, 22.495}, {8.34, 13.88, 22.51}, {8.35, 13.87, 22.525}, {8.36, 13.85, 22.54}, {8.37, 13.84, 22.555}, {8.38, 13.83, 22.57}, {8.39, 13.82, 22.585}, {8.4, 13.81, 22.6}, {8.41, 13.8, 22.615}, {8.42, 13.79, 22.63}, {8.43, 13.77, 22.645}, {8.44, 13.76, 22.66}, {8.45, 13.75, 22.675}, {8.46, 13.74, 22.69}, {8.47, 13.73, 22.705}, {8.48, 13.72, 22.72}, {8.49, 13.71, 22.735}, {8.5, 13.7, 22.75}, {8.51, 13.68, 22.765}, {8.52, 13.67, 22.78}, {8.53, 13.66, 22.795}, {8.54, 13.65, 22.81}, {8.55, 13.64, 22.825}, {8.56, 13.63, 22.84}, {8.57, 13.62, 22.855}, {8.58, 13.61, 22.87}, {8.59, 13.6, 22.885}, {8.6, 13.58, 22.9}, {8.61, 13.57, 22.915}, {8.62, 13.56, 22.93}, {8.63, 13.55, 22.945}, {8.64, 13.54, 22.96}, {8.65, 13.53, 22.975}, {8.66, 13.52, 22.99}, {8.67, 13.51, 23.005}, {8.68, 13.5, 23.02}, {8.69, 13.49, 23.035}, {8.7, 13.48, 23.05}, {8.71, 13.47, 23.065}, {8.72, 13.45, 23.08}, {8.73, 13.44, 23.095}, {8.74, 13.43, 23.11}, {8.75, 13.42, 23.125}, {8.76, 13.41, 23.14}, {8.77, 13.4, 23.155}, {8.78, 13.39, 23.17}, {8.79, 13.38, 23.185}, {8.8, 13.37, 23.2}, {8.81, 13.36, 23.215}, {8.82, 13.35, 23.23}, {8.83, 13.34, 23.245}, {8.84, 13.33, 23.26}, {8.85, 13.32, 23.275}, {8.86, 13.31, 23.29}, {8.87, 13.3, 23.305}, {8.88, 13.29, 23.32}, {8.89, 13.28, 23.335}, {8.9, 13.27, 23.35}, {8.91, 13.25, 23.365}, {8.92, 13.24, 23.38}, {8.93, 13.23, 23.395}, {8.94, 13.22, 23.41}, {8.95, 13.21, 23.425}, {8.96, 13.2, 23.44}, {8.97, 13.19, 23.455}, {8.98, 13.18, 23.47}, {8.99, 13.17, 23.485}, {9.0, 13.16, 23.5}, {9.01, 13.15, 23.515}, {9.02, 13.14, 23.53}, {9.03, 13.13, 23.545}, {9.04, 13.12, 23.56}, {9.05, 13.11, 23.575}, {9.06, 13.1, 23.59}, {9.07, 13.09, 23.605}, {9.08, 13.08, 23.62}, {9.09, 13.07, 23.635}, {9.1, 13.06, 23.65}, {9.11, 13.05, 23.665}, {9.12, 13.04, 23.68}, {9.13, 13.03, 23.695}, {9.14, 13.02, 23.71}, {9.15, 13.01, 23.725}, {9.16, 13.0, 23.74}, {9.17, 12.99, 23.755}, {9.18, 12.98, 23.77}, {9.19, 12.97, 23.785}, {9.2, 12.96, 23.8}, {9.21, 12.95, 23.815}, {9.22, 12.94, 23.83}, {9.23, 12.93, 23.845}, {9.24, 12.92, 23.86}, {9.25, 12.91, 23.875}, {9.26, 12.9, 23.89}, {9.27, 12.89, 23.905}, {9.28, 12.89, 23.92}, {9.29, 12.88, 23.935}, {9.3, 12.87, 23.95}, {9.31, 12.86, 23.965}, {9.32, 12.85, 23.98}, {9.33, 12.84, 23.995}, {9.34, 12.83, 24.01}, {9.35, 12.82, 24.025}, {9.36, 12.81, 24.04}, {9.37, 12.8, 24.055}, {9.38, 12.79, 24.07}, {9.39, 12.78, 24.085}, {9.4, 12.77, 24.1}, {9.41, 12.76, 24.115}, {9.42, 12.75, 24.13}, {9.43, 12.74, 24.145}, {9.44, 12.73, 24.16}, {9.45, 12.72, 24.175}, {9.46, 12.71, 24.19}, {9.47, 12.7, 24.205}, {9.48, 12.7, 24.22}, {9.49, 12.69, 24.235}, {9.5, 12.68, 24.25}, {9.51, 12.67, 24.265}, {9.52, 12.66, 24.28}, {9.53, 12.65, 24.295}, {9.54, 12.64, 24.31}, {9.55, 12.63, 24.325}, {9.56, 12.62, 24.34}, {9.57, 12.61, 24.355}, {9.58, 12.6, 24.37}, {9.59, 12.59, 24.385}, {9.6, 12.58, 24.4}, {9.61, 12.58, 24.415}, {9.62, 12.57, 24.43}, {9.63, 12.56, 24.445}, {9.64, 12.55, 24.46}, {9.65, 12.54, 24.475}, {9.66, 12.53, 24.49}, {9.67, 12.52, 24.505}, {9.68, 12.51, 24.52}, {9.69, 12.5, 24.535}, {9.7, 12.49, 24.55}, {9.71, 12.48, 24.565}, {9.72, 12.48, 24.58}, {9.73, 12.47, 24.595}, {9.74, 12.46, 24.61}, {9.75, 12.45, 24.625}, {9.76, 12.44, 24.64}, {9.77, 12.43, 24.655}, {9.78, 12.42, 24.67}, {9.79, 12.41, 24.685}, {9.8, 12.4, 24.7}, {9.81, 12.4, 24.715}, {9.82, 12.39, 24.73}, {9.83, 12.38, 24.745}, {9.84, 12.37, 24.76}, {9.85, 12.36, 24.775}, {9.86, 12.35, 24.79}, {9.87, 12.34, 24.805}, {9.88, 12.33, 24.82}, {9.89, 12.33, 24.835}, {9.9, 12.32, 24.85}, {9.91, 12.31, 24.865}, {9.92, 12.3, 24.88}, {9.93, 12.29, 24.895}, {9.94, 12.28, 24.91}, {9.95, 12.27, 24.925}, {9.96, 12.26, 24.94}, {9.97, 12.26, 24.955}, {9.98, 12.25, 24.97}, {9.99, 12.24, 24.985}, {10.0, 12.23, 25.0}};            
        // distance in meters of april tag, angle from 0-90 (0 is horiz), shooter mps
    }

}
