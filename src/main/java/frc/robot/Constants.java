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
        TELEOP_DISABLE_SWERVE
    }

    // Global
    public static RobotState robotState = RobotState.DISABLED;

    public static final class Superstructure {
        // Delay between note detected and retract intake
        public static final double kIntakeTimeToStowToleranceSec = 0.25;

        // Elapsed time beam break detecting note before stow
        public static final double kIndexerIntakeRollbackTimeSec = 0.05; // 0.05

        // Elapsed time all elements at wait speaker pos before score speaker
        public static final double kWaitSpeakerTimeToleranceSec = 0.25;

        // Elapsed time shooting into speaker before stow
        public static final double kScoreSpeakerShootTimeToleranceSec = 1.0; // intake fallback = 0.5s,\1.0

        public static final double kScoreSpeakerPivotTimeToleranceSec = 0.5;

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

        public static PIDConst kPID = new PIDConst(0.00675, 0, 0, 0.015);

        public static final double kManualMultiplier = 0.001;

        public static final double kAtGoalTolerance = 0.25;

        public static final double kShooterPivotGearRatio = 148.15;

        public static final double kMaxShooterPivotOutput = 0.3;

        // public static final double kMaxVelocityDeg = 0.0;
        // public static final double kMaxAccelerationDeg = 0.0;

        public static final double kTargetAngleStowed = 5.0;
        public static final double kTargetAngleIntake = 5.0;
        // public static final double kTargetAngleSpeaker = 0.0;
        // public static final double kTargetAngleSpeakerHigh = 0.0;
        public static final double kTargetAngleSpeakerFromAmp = kTargetAngleStowed;
        public static final double kTargetAngleSpeakerFromPodium = kTargetAngleStowed;
        public static final double kTargetAngleSpeakerFromSubwoofer = 40.0;
        public static final double kTargetAngleAmp = kTargetAngleStowed;
        public static final double kTargetAngleSource = kTargetAngleStowed;
    }

    public static final class Shooter {
        public static final boolean kIsShooterTopInverted = true;
        public static final boolean kIsShooterBottomInverted = true;
        public static final boolean kIsIndexerInverted = true;

        public static final double kShooterSpeedScoreSpeakerSubwoofer = 10.0; // 10.0
        public static final double kShooterSpeedScoreSpeakerPodium = 5.0;
        public static final double kShooterSpeedScoreSpeakerAmpZone = 10.0;
        public static final double kShooterSpeedScoreAmp = 1.0;
        public static final double kShooterEjectNoteSpeed = -0.25;

        public static final double kIndexerSpeedIntake = 0.3;
        public static final double kIndexerSpeedScoreSpeaker = 0.9; // 0.9
        public static final double kIndexerSpeedScoreAmp = 0.4;
        public static final double kIndexerEjectNoteSpeed = -0.5;
        // public static final double kIntakeRollbackSpeed = 0.15;

        // JUST IN CASE
        public static final double kIndexerSpeedSource = -0.1;

        public static final double kShooterGearRatio = 0.6666666666;
        public static final double kIndexerGearRatio = 2.0;

        public static final double kShooterWheelDiameter = Conversions.inchesToMeters(2.0);

        public static final double kShooterIncramentMultiplier = 0.01;

        public static final double kHasNoteCurrentThreshold = 36.0;

        public static final double kAtGoalTolerance = 0.1; // 0.05 = precise for dynamic

        public static double kPShooter = 0.25;
        public static double kIShooter = 0.0;
        public static double kDShooter = 0.0075;

        public static final double kPIndexer = 0.0;
        public static final double kIIndexer = 0.0;
        public static final double kDIndexer = 0.0;
    }

    public static final class Intake {
        public static final double kPivotAngleStowed = 20.0; // 20
        public static final double kPivotAngleIntake = 172.0;
        public static final double kScoreSpeaker = 80.0;

        public static final double kRollSpeedStowed = 0.0;
        public static final double kRollSpeedIntake = 0.6;
        public static final double kRollSpeedIntakeRollback = 0.05;
        public static final double kRollSpeedScoreSpeaker = 0.2;
        public static final double kEjectNoteSpeed = -0.6;
        ;

        public static final boolean kIsRollInverted = true;
        public static final boolean kIsPivotInverted = false;

        // public static final double kPivotMaxVelocity = 0.0;
        // public static final double kPivotMaxAccel = 0.0;
        public static final double kMaxPivotOutput = 0.2;

        public static final double kAtGoalTolerance = 5; // 0.5

        public static final double kPivotIntakeGearRatio = 36.0;

        public static final double kPPivot = 0.002914; // 1, 0.1
        public static final double kIPivot = 0.0;
        public static final double kDPivot = 0.00002445;

    }

    public static final class Limelight {
        public static final double kAcceptableVolatilityThreshold = 0.2;

        public static final double kMaxDrivingSpeed = 0.0;
        public static final double kMaxTurningSpeed = 0.0;

        // should go into robotmap
        public static final double limelightMountAngleDegrees = 0.0;
        public static final double goalHeightInches = 0.0;
        public static final double limelightHeightInches = 0.0;
    }

    public static final class Drivetrain {
        // Measurements
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kModuleOffsetMeters = 0.572;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
                new Translation2d(-kModuleOffsetMeters / 2, kModuleOffsetMeters / 2),
                new Translation2d(kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
                new Translation2d(kModuleOffsetMeters / 2, kModuleOffsetMeters / 2));

        // PID
        public static double kPTurnMotorFL = 0.003; // 0.0125
        public static double kITurnMotorFL = 0.00;
        public static double kDTurnMotorFL = 0.0000; // 0.000026

        public static double kPTurnMotorFR = 0.003; // 0.0125
        public static double kITurnMotorFR = 0.00;
        public static double kDTurnMotorFR = 0.0000; // 0.000026

        public static double kPTurnMotorBL = 0.003; // 0.0125
        public static double kITurnMotorBL = 0.00;
        public static double kDTurnMotorBL = 0.0000; // 0.000026

        public static double kPTurnMotorBR = 0.003; // 0.0125
        public static double kITurnMotorBR = 0.00;
        public static double kDTurnMotorBR = 0.0000; // 0.000026
        

        public static final double kPHeading = 1.2; // 0.6
        public static final double kIHeading = 0.0000;
        public static final double kDHeading = 0.0;

        public static final double kTranslationMultPP = 0.2;

        public static final double kPTranslationPP = 3.0; // 4.0 
        public static final double kITranslationPP = 0.0;
        public static final double kDTranslationPP = 0.0;

        public static final double kPRotationPP = 0.0; // 3.8
        public static final double kIRotationPP = 0.0;
        public static final double kDRotationPP = 0.0; // 0.0


        // Speed/Accel
        public static final double kMaxDriveModuleSpeedMPS = 4.0;

        public static double kMaxSpeed = 1.1; // 0.8 //1.0 // 0.6 - latest
        public static double kMaxAccel = 1.6; // 0.7 //1.0 // 1.0 - latest


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
        
        public static double kCANCoderOffsetFrontLeft = 50.3; // 50.3
        public static double kCANCoderOffsetFrontRight = 126.8; // 126.8
        public static double kCANCoderOffsetBackLeft = 270.2; // 270.2
        public static double kCANCoderOffsetBackRight = 161.6; // 161.6


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
            int index = (int) ((dist - 0.9) / 0.005);
            double[] retArr = { vals[index][1], vals[index][2] };
            return retArr;
        }

        // Values below, nothing below vals
        public static final double[][] vals = { { 0.9, 61.83, 11.8 }, { 0.905, 61.7, 11.81 }, { 0.91, 61.57, 11.82 },
                { 0.915, 61.43, 11.83 }, { 0.92, 61.3, 11.84 }, { 0.925, 61.17, 11.85 }, { 0.93, 61.04, 11.86 },
                { 0.935, 60.91, 11.87 }, { 0.94, 60.78, 11.88 }, { 0.945, 60.65, 11.89 }, { 0.95, 60.52, 11.9 },
                { 0.955, 60.39, 11.91 }, { 0.96, 60.26, 11.92 }, { 0.965, 60.13, 11.93 }, { 0.97, 60.0, 11.94 },
                { 0.975, 59.87, 11.95 }, { 0.98, 59.74, 11.96 }, { 0.985, 59.61, 11.97 }, { 0.99, 59.49, 11.98 },
                { 0.995, 59.36, 11.99 }, { 1.0, 59.23, 12.0 }, { 1.005, 59.11, 12.01 }, { 1.01, 58.98, 12.02 },
                { 1.015, 58.85, 12.03 }, { 1.02, 58.73, 12.04 }, { 1.025, 58.6, 12.05 }, { 1.03, 58.48, 12.06 },
                { 1.035, 58.36, 12.07 }, { 1.04, 58.23, 12.08 }, { 1.045, 58.11, 12.09 }, { 1.05, 57.98, 12.1 },
                { 1.055, 57.86, 12.11 }, { 1.06, 57.74, 12.12 }, { 1.065, 57.62, 12.13 }, { 1.07, 57.5, 12.14 },
                { 1.075, 57.37, 12.15 }, { 1.08, 57.25, 12.16 }, { 1.085, 57.13, 12.17 }, { 1.09, 57.01, 12.18 },
                { 1.095, 56.89, 12.19 }, { 1.1, 56.77, 12.2 }, { 1.105, 56.65, 12.21 }, { 1.11, 56.53, 12.22 },
                { 1.115, 56.41, 12.23 }, { 1.12, 56.3, 12.24 }, { 1.125, 56.18, 12.25 }, { 1.13, 56.06, 12.26 },
                { 1.134, 55.97, 12.268 }, { 1.14, 55.83, 12.28 }, { 1.144, 55.73, 12.288 }, { 1.15, 55.59, 12.3 },
                { 1.154, 55.5, 12.308 }, { 1.16, 55.36, 12.32 }, { 1.164, 55.27, 12.328 }, { 1.17, 55.13, 12.34 },
                { 1.174, 55.04, 12.348 }, { 1.18, 54.9, 12.36 }, { 1.184, 54.81, 12.368 }, { 1.19, 54.68, 12.38 },
                { 1.194, 54.58, 12.388 }, { 1.2, 54.45, 12.4 }, { 1.204, 54.36, 12.408 }, { 1.21, 54.22, 12.42 },
                { 1.214, 54.14, 12.428 }, { 1.22, 54.0, 12.44 }, { 1.224, 53.91, 12.448 }, { 1.23, 53.78, 12.46 },
                { 1.234, 53.69, 12.468 }, { 1.24, 53.56, 12.48 }, { 1.244, 53.47, 12.488 }, { 1.25, 53.34, 12.5 },
                { 1.255, 53.23, 12.51 }, { 1.26, 53.12, 12.52 }, { 1.265, 53.01, 12.53 }, { 1.27, 52.9, 12.54 },
                { 1.275, 52.8, 12.55 }, { 1.28, 52.69, 12.56 }, { 1.285, 52.58, 12.57 }, { 1.29, 52.47, 12.58 },
                { 1.295, 52.37, 12.59 }, { 1.3, 52.26, 12.6 }, { 1.305, 52.16, 12.61 }, { 1.31, 52.05, 12.62 },
                { 1.315, 51.95, 12.63 }, { 1.32, 51.84, 12.64 }, { 1.325, 51.74, 12.65 }, { 1.33, 51.63, 12.66 },
                { 1.335, 51.53, 12.67 }, { 1.34, 51.42, 12.68 }, { 1.345, 51.32, 12.69 }, { 1.35, 51.22, 12.7 },
                { 1.355, 51.11, 12.71 }, { 1.36, 51.01, 12.72 }, { 1.365, 50.91, 12.73 }, { 1.37, 50.81, 12.74 },
                { 1.375, 50.71, 12.75 }, { 1.38, 50.61, 12.76 }, { 1.384, 50.53, 12.768 }, { 1.39, 50.4, 12.78 },
                { 1.394, 50.32, 12.788 }, { 1.4, 50.2, 12.8 }, { 1.404, 50.12, 12.808 }, { 1.41, 50.01, 12.82 },
                { 1.414, 49.93, 12.828 }, { 1.42, 49.81, 12.84 }, { 1.424, 49.73, 12.848 }, { 1.43, 49.61, 12.86 },
                { 1.434, 49.53, 12.868 }, { 1.44, 49.42, 12.88 }, { 1.444, 49.34, 12.888 }, { 1.45, 49.22, 12.9 },
                { 1.454, 49.15, 12.908 }, { 1.46, 49.03, 12.92 }, { 1.464, 48.95, 12.928 }, { 1.47, 48.84, 12.94 },
                { 1.474, 48.76, 12.948 }, { 1.48, 48.65, 12.96 }, { 1.484, 48.57, 12.968 }, { 1.49, 48.46, 12.98 },
                { 1.494, 48.38, 12.988 }, { 1.5, 48.27, 13.0 }, { 1.505, 48.18, 13.01 }, { 1.51, 48.08, 13.02 },
                { 1.515, 47.99, 13.03 }, { 1.52, 47.9, 13.04 }, { 1.525, 47.81, 13.05 }, { 1.53, 47.72, 13.06 },
                { 1.535, 47.62, 13.07 }, { 1.54, 47.53, 13.08 }, { 1.545, 47.44, 13.09 }, { 1.55, 47.35, 13.1 },
                { 1.555, 47.26, 13.11 }, { 1.56, 47.17, 13.12 }, { 1.565, 47.08, 13.13 }, { 1.57, 46.99, 13.14 },
                { 1.575, 46.9, 13.15 }, { 1.58, 46.81, 13.16 }, { 1.585, 46.72, 13.17 }, { 1.59, 46.63, 13.18 },
                { 1.595, 46.55, 13.19 }, { 1.6, 46.46, 13.2 }, { 1.605, 46.37, 13.21 }, { 1.61, 46.28, 13.22 },
                { 1.615, 46.19, 13.23 }, { 1.62, 46.11, 13.24 }, { 1.625, 46.02, 13.25 }, { 1.63, 45.93, 13.26 },
                { 1.634, 45.87, 13.268 }, { 1.64, 45.76, 13.28 }, { 1.644, 45.69, 13.288 }, { 1.65, 45.59, 13.3 },
                { 1.654, 45.52, 13.308 }, { 1.66, 45.42, 13.32 }, { 1.664, 45.35, 13.328 }, { 1.67, 45.25, 13.34 },
                { 1.674, 45.19, 13.348 }, { 1.68, 45.09, 13.36 }, { 1.684, 45.02, 13.368 }, { 1.69, 44.92, 13.38 },
                { 1.694, 44.85, 13.388 }, { 1.7, 44.75, 13.4 }, { 1.704, 44.69, 13.408 }, { 1.71, 44.59, 13.42 },
                { 1.714, 44.52, 13.428 }, { 1.72, 44.43, 13.44 }, { 1.724, 44.36, 13.448 }, { 1.73, 44.26, 13.46 },
                { 1.734, 44.2, 13.468 }, { 1.74, 44.1, 13.48 }, { 1.744, 44.04, 13.488 }, { 1.75, 43.94, 13.5 },
                { 1.755, 43.86, 13.51 }, { 1.76, 43.78, 13.52 }, { 1.765, 43.7, 13.53 }, { 1.77, 43.62, 13.54 },
                { 1.775, 43.55, 13.55 }, { 1.78, 43.47, 13.56 }, { 1.785, 43.39, 13.57 }, { 1.79, 43.31, 13.58 },
                { 1.795, 43.23, 13.59 }, { 1.8, 43.16, 13.6 }, { 1.805, 43.08, 13.61 }, { 1.81, 43.0, 13.62 },
                { 1.815, 42.92, 13.63 }, { 1.82, 42.85, 13.64 }, { 1.825, 42.77, 13.65 }, { 1.83, 42.69, 13.66 },
                { 1.835, 42.62, 13.67 }, { 1.84, 42.54, 13.68 }, { 1.845, 42.47, 13.69 }, { 1.85, 42.39, 13.7 },
                { 1.855, 42.32, 13.71 }, { 1.86, 42.24, 13.72 }, { 1.865, 42.17, 13.73 }, { 1.87, 42.09, 13.74 },
                { 1.875, 42.02, 13.75 }, { 1.88, 41.95, 13.76 }, { 1.884, 41.89, 13.768 }, { 1.89, 41.8, 13.78 },
                { 1.894, 41.74, 13.788 }, { 1.9, 41.65, 13.8 }, { 1.904, 41.6, 13.808 }, { 1.91, 41.51, 13.82 },
                { 1.914, 41.45, 13.828 }, { 1.92, 41.37, 13.84 }, { 1.924, 41.31, 13.848 }, { 1.93, 41.22, 13.86 },
                { 1.934, 41.16, 13.868 }, { 1.94, 41.08, 13.88 }, { 1.944, 41.02, 13.888 }, { 1.95, 40.94, 13.9 },
                { 1.954, 40.88, 13.908 }, { 1.96, 40.8, 13.92 }, { 1.964, 40.74, 13.928 }, { 1.97, 40.66, 13.94 },
                { 1.974, 40.6, 13.948 }, { 1.98, 40.52, 13.96 }, { 1.984, 40.46, 13.968 }, { 1.99, 40.38, 13.98 },
                { 1.994, 40.33, 13.988 }, { 2.0, 40.24, 14.0 }, { 2.005, 40.18, 14.01 }, { 2.01, 40.11, 14.02 },
                { 2.014, 40.05, 14.028 }, { 2.02, 39.97, 14.04 }, { 2.025, 39.91, 14.05 }, { 2.03, 39.84, 14.06 },
                { 2.034, 39.78, 14.068 }, { 2.04, 39.7, 14.08 }, { 2.045, 39.64, 14.09 }, { 2.05, 39.57, 14.1 },
                { 2.054, 39.52, 14.108 }, { 2.06, 39.44, 14.12 }, { 2.065, 39.37, 14.13 }, { 2.07, 39.31, 14.14 },
                { 2.074, 39.26, 14.148 }, { 2.08, 39.18, 14.16 }, { 2.085, 39.11, 14.17 }, { 2.09, 39.05, 14.18 },
                { 2.094, 39.0, 14.188 }, { 2.1, 38.92, 14.2 }, { 2.105, 38.86, 14.21 }, { 2.11, 38.79, 14.22 },
                { 2.114, 38.74, 14.228 }, { 2.12, 38.66, 14.24 }, { 2.125, 38.6, 14.25 }, { 2.13, 38.54, 14.26 },
                { 2.135, 38.47, 14.27 }, { 2.14, 38.41, 14.28 }, { 2.145, 38.35, 14.29 }, { 2.15, 38.29, 14.3 },
                { 2.155, 38.22, 14.31 }, { 2.16, 38.16, 14.32 }, { 2.165, 38.1, 14.33 }, { 2.17, 38.04, 14.34 },
                { 2.175, 37.98, 14.35 }, { 2.18, 37.92, 14.36 }, { 2.185, 37.86, 14.37 }, { 2.19, 37.79, 14.38 },
                { 2.195, 37.73, 14.39 }, { 2.2, 37.67, 14.4 }, { 2.205, 37.61, 14.41 }, { 2.21, 37.55, 14.42 },
                { 2.215, 37.49, 14.43 }, { 2.22, 37.43, 14.44 }, { 2.225, 37.37, 14.45 }, { 2.23, 37.31, 14.46 },
                { 2.235, 37.25, 14.47 }, { 2.24, 37.2, 14.48 }, { 2.245, 37.14, 14.49 }, { 2.25, 37.08, 14.5 },
                { 2.255, 37.02, 14.51 }, { 2.26, 36.96, 14.52 }, { 2.264, 36.91, 14.528 }, { 2.27, 36.84, 14.54 },
                { 2.275, 36.79, 14.55 }, { 2.28, 36.73, 14.56 }, { 2.284, 36.68, 14.568 }, { 2.29, 36.61, 14.58 },
                { 2.295, 36.56, 14.59 }, { 2.3, 36.5, 14.6 }, { 2.304, 36.45, 14.608 }, { 2.31, 36.39, 14.62 },
                { 2.315, 36.33, 14.63 }, { 2.32, 36.27, 14.64 }, { 2.324, 36.23, 14.648 }, { 2.33, 36.16, 14.66 },
                { 2.335, 36.11, 14.67 }, { 2.34, 36.05, 14.68 }, { 2.344, 36.01, 14.688 }, { 2.35, 35.94, 14.7 },
                { 2.355, 35.88, 14.71 }, { 2.36, 35.83, 14.72 }, { 2.364, 35.79, 14.728 }, { 2.37, 35.72, 14.74 },
                { 2.375, 35.67, 14.75 }, { 2.38, 35.61, 14.76 }, { 2.385, 35.56, 14.77 }, { 2.39, 35.5, 14.78 },
                { 2.395, 35.45, 14.79 }, { 2.4, 35.4, 14.8 }, { 2.405, 35.34, 14.81 }, { 2.41, 35.29, 14.82 },
                { 2.415, 35.23, 14.83 }, { 2.42, 35.18, 14.84 }, { 2.425, 35.13, 14.85 }, { 2.43, 35.08, 14.86 },
                { 2.435, 35.02, 14.87 }, { 2.44, 34.97, 14.88 }, { 2.445, 34.92, 14.89 }, { 2.45, 34.87, 14.9 },
                { 2.455, 34.81, 14.91 }, { 2.46, 34.76, 14.92 }, { 2.465, 34.71, 14.93 }, { 2.47, 34.66, 14.94 },
                { 2.475, 34.61, 14.95 }, { 2.48, 34.56, 14.96 }, { 2.485, 34.51, 14.97 }, { 2.49, 34.45, 14.98 },
                { 2.495, 34.4, 14.99 }, { 2.5, 34.35, 15.0 }, { 2.505, 34.3, 15.01 }, { 2.51, 34.25, 15.02 },
                { 2.514, 34.21, 15.028 }, { 2.52, 34.15, 15.04 }, { 2.525, 34.1, 15.05 }, { 2.53, 34.05, 15.06 },
                { 2.534, 34.01, 15.068 }, { 2.54, 33.95, 15.08 }, { 2.545, 33.9, 15.09 }, { 2.55, 33.85, 15.1 },
                { 2.554, 33.81, 15.108 }, { 2.56, 33.76, 15.12 }, { 2.565, 33.71, 15.13 }, { 2.57, 33.66, 15.14 },
                { 2.574, 33.62, 15.148 }, { 2.58, 33.56, 15.16 }, { 2.585, 33.51, 15.17 }, { 2.59, 33.46, 15.18 },
                { 2.594, 33.43, 15.188 }, { 2.6, 33.37, 15.2 }, { 2.605, 33.32, 15.21 }, { 2.61, 33.27, 15.22 },
                { 2.614, 33.24, 15.228 }, { 2.62, 33.18, 15.24 }, { 2.625, 33.13, 15.25 }, { 2.63, 33.08, 15.26 },
                { 2.635, 33.04, 15.27 }, { 2.64, 32.99, 15.28 }, { 2.645, 32.94, 15.29 }, { 2.65, 32.9, 15.3 },
                { 2.655, 32.85, 15.31 }, { 2.66, 32.8, 15.32 }, { 2.665, 32.76, 15.33 }, { 2.67, 32.71, 15.34 },
                { 2.675, 32.67, 15.35 }, { 2.68, 32.62, 15.36 }, { 2.685, 32.58, 15.37 }, { 2.69, 32.53, 15.38 },
                { 2.695, 32.48, 15.39 }, { 2.7, 32.44, 15.4 }, { 2.705, 32.39, 15.41 }, { 2.71, 32.35, 15.42 },
                { 2.715, 32.3, 15.43 }, { 2.72, 32.26, 15.44 }, { 2.725, 32.21, 15.45 }, { 2.73, 32.17, 15.46 },
                { 2.735, 32.13, 15.47 }, { 2.74, 32.08, 15.48 }, { 2.745, 32.04, 15.49 }, { 2.75, 31.99, 15.5 },
                { 2.755, 31.95, 15.51 }, { 2.76, 31.91, 15.52 }, { 2.764, 31.87, 15.528 }, { 2.77, 31.82, 15.54 },
                { 2.775, 31.78, 15.55 }, { 2.78, 31.73, 15.56 }, { 2.784, 31.7, 15.568 }, { 2.79, 31.65, 15.58 },
                { 2.795, 31.6, 15.59 }, { 2.8, 31.56, 15.6 }, { 2.804, 31.53, 15.608 }, { 2.81, 31.47, 15.62 },
                { 2.815, 31.43, 15.63 }, { 2.82, 31.39, 15.64 }, { 2.824, 31.36, 15.648 }, { 2.83, 31.31, 15.66 },
                { 2.835, 31.26, 15.67 }, { 2.84, 31.22, 15.68 }, { 2.844, 31.19, 15.688 }, { 2.85, 31.14, 15.7 },
                { 2.855, 31.1, 15.71 }, { 2.86, 31.05, 15.72 }, { 2.864, 31.02, 15.728 }, { 2.87, 30.97, 15.74 },
                { 2.875, 30.93, 15.75 }, { 2.88, 30.89, 15.76 }, { 2.885, 30.85, 15.77 }, { 2.89, 30.81, 15.78 },
                { 2.895, 30.77, 15.79 }, { 2.9, 30.73, 15.8 }, { 2.905, 30.69, 15.81 }, { 2.91, 30.65, 15.82 },
                { 2.915, 30.61, 15.83 }, { 2.92, 30.57, 15.84 }, { 2.925, 30.53, 15.85 }, { 2.93, 30.49, 15.86 },
                { 2.935, 30.45, 15.87 }, { 2.94, 30.41, 15.88 }, { 2.945, 30.37, 15.89 }, { 2.95, 30.33, 15.9 },
                { 2.955, 30.29, 15.91 }, { 2.96, 30.25, 15.92 }, { 2.965, 30.21, 15.93 }, { 2.97, 30.17, 15.94 },
                { 2.975, 30.13, 15.95 }, { 2.98, 30.09, 15.96 }, { 2.985, 30.05, 15.97 }, { 2.99, 30.01, 15.98 },
                { 2.995, 29.98, 15.99 }, { 3.0, 29.94, 16.0 }, { 3.005, 29.9, 16.01 }, { 3.01, 29.86, 16.02 },
                { 3.014, 29.83, 16.028 }, { 3.02, 29.78, 16.04 }, { 3.025, 29.75, 16.05 }, { 3.03, 29.71, 16.06 },
                { 3.034, 29.68, 16.068 }, { 3.04, 29.63, 16.08 }, { 3.045, 29.6, 16.09 }, { 3.05, 29.56, 16.1 },
                { 3.054, 29.53, 16.108 }, { 3.06, 29.48, 16.12 }, { 3.065, 29.45, 16.13 }, { 3.07, 29.41, 16.14 },
                { 3.074, 29.38, 16.148 }, { 3.08, 29.33, 16.16 }, { 3.085, 29.3, 16.17 }, { 3.09, 29.26, 16.18 },
                { 3.094, 29.23, 16.188 }, { 3.1, 29.19, 16.2 }, { 3.105, 29.15, 16.21 }, { 3.11, 29.12, 16.22 },
                { 3.114, 29.09, 16.228 }, { 3.12, 29.04, 16.24 }, { 3.125, 29.01, 16.25 }, { 3.13, 28.97, 16.26 },
                { 3.135, 28.93, 16.27 }, { 3.14, 28.9, 16.28 }, { 3.145, 28.86, 16.29 }, { 3.15, 28.83, 16.3 },
                { 3.155, 28.79, 16.31 }, { 3.16, 28.76, 16.32 }, { 3.165, 28.72, 16.33 }, { 3.17, 28.69, 16.34 },
                { 3.175, 28.65, 16.35 }, { 3.18, 28.62, 16.36 }, { 3.185, 28.58, 16.37 }, { 3.19, 28.55, 16.38 },
                { 3.195, 28.51, 16.39 }, { 3.2, 28.48, 16.4 }, { 3.205, 28.44, 16.41 }, { 3.21, 28.41, 16.42 },
                { 3.215, 28.37, 16.43 }, { 3.22, 28.34, 16.44 }, { 3.225, 28.3, 16.45 }, { 3.23, 28.27, 16.46 },
                { 3.235, 28.24, 16.47 }, { 3.24, 28.2, 16.48 }, { 3.245, 28.17, 16.49 }, { 3.25, 28.13, 16.5 },
                { 3.255, 28.1, 16.51 }, { 3.26, 28.07, 16.52 }, { 3.264, 28.04, 16.528 }, { 3.27, 28.0, 16.54 },
                { 3.275, 27.97, 16.55 }, { 3.28, 27.93, 16.56 }, { 3.284, 27.91, 16.568 }, { 3.29, 27.87, 16.58 },
                { 3.295, 27.83, 16.59 }, { 3.3, 27.8, 16.6 }, { 3.304, 27.77, 16.608 }, { 3.31, 27.73, 16.62 },
                { 3.315, 27.7, 16.63 }, { 3.32, 27.67, 16.64 }, { 3.324, 27.64, 16.648 }, { 3.33, 27.6, 16.66 },
                { 3.335, 27.57, 16.67 }, { 3.34, 27.54, 16.68 }, { 3.344, 27.51, 16.688 }, { 3.35, 27.47, 16.7 },
                { 3.355, 27.44, 16.71 }, { 3.36, 27.41, 16.72 }, { 3.364, 27.38, 16.728 }, { 3.37, 27.35, 16.74 },
                { 3.375, 27.31, 16.75 }, { 3.38, 27.28, 16.76 }, { 3.385, 27.25, 16.77 }, { 3.39, 27.22, 16.78 },
                { 3.395, 27.19, 16.79 }, { 3.4, 27.16, 16.8 }, { 3.405, 27.12, 16.81 }, { 3.41, 27.09, 16.82 },
                { 3.415, 27.06, 16.83 }, { 3.42, 27.03, 16.84 }, { 3.425, 27.0, 16.85 }, { 3.43, 26.97, 16.86 },
                { 3.435, 26.94, 16.87 }, { 3.44, 26.91, 16.88 }, { 3.445, 26.88, 16.89 }, { 3.45, 26.84, 16.9 },
                { 3.455, 26.81, 16.91 }, { 3.46, 26.78, 16.92 }, { 3.465, 26.75, 16.93 }, { 3.47, 26.72, 16.94 },
                { 3.475, 26.69, 16.95 }, { 3.48, 26.66, 16.96 }, { 3.485, 26.63, 16.97 }, { 3.49, 26.6, 16.98 },
                { 3.495, 26.57, 16.99 }, { 3.5, 26.54, 17.0 }, { 3.505, 26.51, 17.01 }, { 3.51, 26.48, 17.02 },
                { 3.514, 26.46, 17.028 }, { 3.52, 26.42, 17.04 }, { 3.525, 26.39, 17.05 }, { 3.53, 26.36, 17.06 },
                { 3.534, 26.34, 17.068 }, { 3.54, 26.3, 17.08 }, { 3.545, 26.27, 17.09 }, { 3.55, 26.25, 17.1 },
                { 3.554, 26.22, 17.108 }, { 3.56, 26.19, 17.12 }, { 3.565, 26.16, 17.13 }, { 3.57, 26.13, 17.14 },
                { 3.574, 26.11, 17.148 }, { 3.58, 26.07, 17.16 }, { 3.585, 26.04, 17.17 }, { 3.59, 26.01, 17.18 },
                { 3.594, 25.99, 17.188 }, { 3.6, 25.96, 17.2 }, { 3.605, 25.93, 17.21 }, { 3.61, 25.9, 17.22 },
                { 3.614, 25.88, 17.228 }, { 3.62, 25.84, 17.24 }, { 3.625, 25.81, 17.25 }, { 3.63, 25.79, 17.26 },
                { 3.635, 25.76, 17.27 }, { 3.64, 25.73, 17.28 }, { 3.645, 25.7, 17.29 }, { 3.65, 25.67, 17.3 },
                { 3.655, 25.65, 17.31 }, { 3.66, 25.62, 17.32 }, { 3.665, 25.59, 17.33 }, { 3.67, 25.56, 17.34 },
                { 3.675, 25.53, 17.35 }, { 3.68, 25.51, 17.36 }, { 3.685, 25.48, 17.37 }, { 3.69, 25.45, 17.38 },
                { 3.695, 25.42, 17.39 }, { 3.7, 25.4, 17.4 }, { 3.705, 25.37, 17.41 }, { 3.71, 25.34, 17.42 },
                { 3.715, 25.32, 17.43 }, { 3.72, 25.29, 17.44 }, { 3.725, 25.26, 17.45 }, { 3.73, 25.23, 17.46 },
                { 3.735, 25.21, 17.47 }, { 3.74, 25.18, 17.48 }, { 3.745, 25.15, 17.49 }, { 3.75, 25.13, 17.5 },
                { 3.755, 25.1, 17.51 }, { 3.76, 25.07, 17.52 }, { 3.764, 25.05, 17.528 }, { 3.77, 25.02, 17.54 },
                { 3.775, 24.99, 17.55 }, { 3.78, 24.97, 17.56 }, { 3.784, 24.95, 17.568 }, { 3.79, 24.92, 17.58 },
                { 3.795, 24.89, 17.59 }, { 3.8, 24.86, 17.6 }, { 3.804, 24.84, 17.608 }, { 3.81, 24.81, 17.62 },
                { 3.815, 24.78, 17.63 }, { 3.82, 24.76, 17.64 }, { 3.824, 24.74, 17.648 }, { 3.83, 24.71, 17.66 },
                { 3.835, 24.68, 17.67 }, { 3.84, 24.66, 17.68 }, { 3.844, 24.64, 17.688 }, { 3.85, 24.6, 17.7 },
                { 3.855, 24.58, 17.71 }, { 3.86, 24.55, 17.72 }, { 3.864, 24.53, 17.728 }, { 3.87, 24.5, 17.74 },
                { 3.875, 24.48, 17.75 }, { 3.88, 24.45, 17.76 }, { 3.885, 24.43, 17.77 }, { 3.89, 24.4, 17.78 },
                { 3.895, 24.38, 17.79 }, { 3.9, 24.35, 17.8 }, { 3.905, 24.33, 17.81 }, { 3.91, 24.3, 17.82 },
                { 3.915, 24.28, 17.83 }, { 3.92, 24.25, 17.84 }, { 3.925, 24.23, 17.85 }, { 3.93, 24.2, 17.86 },
                { 3.935, 24.18, 17.87 }, { 3.94, 24.15, 17.88 }, { 3.945, 24.13, 17.89 }, { 3.95, 24.1, 17.9 },
                { 3.955, 24.08, 17.91 }, { 3.96, 24.06, 17.92 }, { 3.965, 24.03, 17.93 }, { 3.97, 24.01, 17.94 },
                { 3.975, 23.98, 17.95 }, { 3.98, 23.96, 17.96 }, { 3.985, 23.94, 17.97 }, { 3.99, 23.91, 17.98 },
                { 3.995, 23.89, 17.99 }, { 4.0, 23.86, 18.0 }, { 4.005, 23.84, 18.01 }, { 4.01, 23.82, 18.02 },
                { 4.015, 23.79, 18.03 }, { 4.02, 23.77, 18.04 }, { 4.024, 23.75, 18.048 }, { 4.03, 23.72, 18.06 },
                { 4.035, 23.7, 18.07 }, { 4.04, 23.67, 18.08 }, { 4.045, 23.65, 18.09 }, { 4.05, 23.63, 18.1 },
                { 4.055, 23.6, 18.11 }, { 4.06, 23.58, 18.12 }, { 4.064, 23.56, 18.128 }, { 4.07, 23.53, 18.14 },
                { 4.075, 23.51, 18.15 }, { 4.08, 23.49, 18.16 }, { 4.085, 23.46, 18.17 }, { 4.09, 23.44, 18.18 },
                { 4.095, 23.42, 18.19 }, { 4.1, 23.39, 18.2 }, { 4.104, 23.38, 18.208 }, { 4.11, 23.35, 18.22 },
                { 4.115, 23.33, 18.23 }, { 4.12, 23.3, 18.24 }, { 4.125, 23.28, 18.25 }, { 4.13, 23.26, 18.26 },
                { 4.135, 23.24, 18.27 }, { 4.14, 23.21, 18.28 }, { 4.145, 23.19, 18.29 }, { 4.15, 23.17, 18.3 },
                { 4.155, 23.15, 18.31 }, { 4.16, 23.12, 18.32 }, { 4.165, 23.1, 18.33 }, { 4.17, 23.08, 18.34 },
                { 4.175, 23.06, 18.35 }, { 4.18, 23.03, 18.36 }, { 4.185, 23.01, 18.37 }, { 4.19, 22.99, 18.38 },
                { 4.195, 22.97, 18.39 }, { 4.2, 22.95, 18.4 }, { 4.205, 22.92, 18.41 }, { 4.21, 22.9, 18.42 },
                { 4.215, 22.88, 18.43 }, { 4.22, 22.86, 18.44 }, { 4.225, 22.84, 18.45 }, { 4.23, 22.81, 18.46 },
                { 4.235, 22.79, 18.47 }, { 4.24, 22.77, 18.48 }, { 4.245, 22.75, 18.49 }, { 4.25, 22.73, 18.5 },
                { 4.255, 22.71, 18.51 }, { 4.26, 22.69, 18.52 }, { 4.265, 22.66, 18.53 }, { 4.27, 22.64, 18.54 },
                { 4.274, 22.63, 18.548 }, { 4.28, 22.6, 18.56 }, { 4.285, 22.58, 18.57 }, { 4.29, 22.56, 18.58 },
                { 4.295, 22.54, 18.59 }, { 4.3, 22.51, 18.6 }, { 4.305, 22.49, 18.61 }, { 4.31, 22.47, 18.62 },
                { 4.314, 22.46, 18.628 }, { 4.32, 22.43, 18.64 }, { 4.325, 22.41, 18.65 }, { 4.33, 22.39, 18.66 },
                { 4.335, 22.37, 18.67 }, { 4.34, 22.35, 18.68 }, { 4.345, 22.33, 18.69 }, { 4.35, 22.31, 18.7 },
                { 4.354, 22.29, 18.708 }, { 4.36, 22.26, 18.72 }, { 4.365, 22.24, 18.73 }, { 4.37, 22.22, 18.74 },
                { 4.375, 22.2, 18.75 }, { 4.38, 22.18, 18.76 }, { 4.385, 22.16, 18.77 }, { 4.39, 22.14, 18.78 },
                { 4.395, 22.12, 18.79 }, { 4.4, 22.1, 18.8 }, { 4.405, 22.08, 18.81 }, { 4.41, 22.06, 18.82 },
                { 4.415, 22.04, 18.83 }, { 4.42, 22.02, 18.84 }, { 4.425, 22.0, 18.85 }, { 4.43, 21.98, 18.86 },
                { 4.435, 21.96, 18.87 }, { 4.44, 21.94, 18.88 }, { 4.445, 21.92, 18.89 }, { 4.45, 21.9, 18.9 },
                { 4.455, 21.88, 18.91 }, { 4.46, 21.86, 18.92 }, { 4.465, 21.84, 18.93 }, { 4.47, 21.82, 18.94 },
                { 4.475, 21.8, 18.95 }, { 4.48, 21.78, 18.96 }, { 4.485, 21.76, 18.97 }, { 4.49, 21.74, 18.98 },
                { 4.495, 21.72, 18.99 }, { 4.5, 21.7, 19.0 }, { 4.505, 21.68, 19.01 }, { 4.51, 21.66, 19.02 },
                { 4.515, 21.65, 19.03 }, { 4.52, 21.63, 19.04 }, { 4.524, 21.61, 19.048 }, { 4.53, 21.59, 19.06 },
                { 4.535, 21.57, 19.07 }, { 4.54, 21.55, 19.08 }, { 4.545, 21.53, 19.09 }, { 4.55, 21.51, 19.1 },
                { 4.555, 21.49, 19.11 }, { 4.56, 21.47, 19.12 }, { 4.564, 21.46, 19.128 }, { 4.57, 21.43, 19.14 },
                { 4.575, 21.42, 19.15 }, { 4.58, 21.4, 19.16 }, { 4.585, 21.38, 19.17 }, { 4.59, 21.36, 19.18 },
                { 4.595, 21.34, 19.19 }, { 4.6, 21.32, 19.2 }, { 4.604, 21.31, 19.208 }, { 4.61, 21.28, 19.22 },
                { 4.615, 21.26, 19.23 }, { 4.62, 21.25, 19.24 }, { 4.625, 21.23, 19.25 }, { 4.63, 21.21, 19.26 },
                { 4.635, 21.19, 19.27 }, { 4.64, 21.17, 19.28 }, { 4.645, 21.15, 19.29 }, { 4.65, 21.14, 19.3 },
                { 4.655, 21.12, 19.31 }, { 4.66, 21.1, 19.32 }, { 4.665, 21.08, 19.33 }, { 4.67, 21.06, 19.34 },
                { 4.675, 21.04, 19.35 }, { 4.68, 21.03, 19.36 }, { 4.685, 21.01, 19.37 }, { 4.69, 20.99, 19.38 },
                { 4.695, 20.97, 19.39 }, { 4.7, 20.95, 19.4 }, { 4.705, 20.93, 19.41 }, { 4.71, 20.92, 19.42 },
                { 4.715, 20.9, 19.43 }, { 4.72, 20.88, 19.44 }, { 4.725, 20.86, 19.45 }, { 4.73, 20.85, 19.46 },
                { 4.735, 20.83, 19.47 }, { 4.74, 20.81, 19.48 }, { 4.745, 20.79, 19.49 }, { 4.75, 20.77, 19.5 },
                { 4.755, 20.76, 19.51 }, { 4.76, 20.74, 19.52 }, { 4.765, 20.72, 19.53 }, { 4.77, 20.7, 19.54 },
                { 4.774, 20.69, 19.548 }, { 4.78, 20.67, 19.56 }, { 4.785, 20.65, 19.57 }, { 4.79, 20.63, 19.58 },
                { 4.795, 20.62, 19.59 }, { 4.8, 20.6, 19.6 }, { 4.805, 20.58, 19.61 }, { 4.81, 20.56, 19.62 },
                { 4.814, 20.55, 19.628 }, { 4.82, 20.53, 19.64 }, { 4.825, 20.51, 19.65 }, { 4.83, 20.49, 19.66 },
                { 4.835, 20.48, 19.67 }, { 4.84, 20.46, 19.68 }, { 4.845, 20.44, 19.69 }, { 4.85, 20.43, 19.7 },
                { 4.854, 20.41, 19.708 }, { 4.86, 20.39, 19.72 }, { 4.865, 20.37, 19.73 }, { 4.87, 20.36, 19.74 },
                { 4.875, 20.34, 19.75 }, { 4.88, 20.32, 19.76 }, { 4.885, 20.31, 19.77 }, { 4.89, 20.29, 19.78 },
                { 4.895, 20.27, 19.79 }, { 4.9, 20.26, 19.8 }, { 4.905, 20.24, 19.81 }, { 4.91, 20.22, 19.82 },
                { 4.915, 20.21, 19.83 }, { 4.92, 20.19, 19.84 }, { 4.925, 20.17, 19.85 }, { 4.93, 20.16, 19.86 },
                { 4.935, 20.14, 19.87 }, { 4.94, 20.12, 19.88 }, { 4.945, 20.11, 19.89 }, { 4.95, 20.09, 19.9 },
                { 4.955, 20.07, 19.91 }, { 4.96, 20.06, 19.92 }, { 4.965, 20.04, 19.93 }, { 4.97, 20.02, 19.94 },
                { 4.975, 20.01, 19.95 }, { 4.98, 19.99, 19.96 }, { 4.985, 19.98, 19.97 }, { 4.99, 19.96, 19.98 },
                { 4.995, 19.94, 19.99 }, { 5.0, 19.93, 20.0 }, { 5.005, 19.91, 20.01 }, { 5.01, 19.89, 20.02 },
                { 5.015, 19.88, 20.03 }, { 5.02, 19.86, 20.04 }, { 5.024, 19.85, 20.048 }, { 5.03, 19.83, 20.06 },
                { 5.035, 19.81, 20.07 }, { 5.04, 19.8, 20.08 }, { 5.045, 19.78, 20.09 }, { 5.05, 19.77, 20.1 },
                { 5.055, 19.75, 20.11 }, { 5.06, 19.73, 20.12 }, { 5.064, 19.72, 20.128 }, { 5.07, 19.7, 20.14 },
                { 5.075, 19.69, 20.15 }, { 5.08, 19.67, 20.16 }, { 5.085, 19.66, 20.17 }, { 5.09, 19.64, 20.18 },
                { 5.095, 19.62, 20.19 }, { 5.1, 19.61, 20.2 }, { 5.104, 19.6, 20.208 }, { 5.11, 19.58, 20.22 },
                { 5.115, 19.56, 20.23 }, { 5.12, 19.55, 20.24 }, { 5.125, 19.53, 20.25 }, { 5.13, 19.52, 20.26 },
                { 5.135, 19.5, 20.27 }, { 5.14, 19.49, 20.28 }, { 5.145, 19.47, 20.29 }, { 5.15, 19.45, 20.3 },
                { 5.155, 19.44, 20.31 }, { 5.16, 19.42, 20.32 }, { 5.165, 19.41, 20.33 }, { 5.17, 19.39, 20.34 },
                { 5.175, 19.38, 20.35 }, { 5.18, 19.36, 20.36 }, { 5.185, 19.35, 20.37 }, { 5.19, 19.33, 20.38 },
                { 5.195, 19.32, 20.39 }, { 5.2, 19.3, 20.4 }, { 5.205, 19.29, 20.41 }, { 5.21, 19.27, 20.42 },
                { 5.215, 19.26, 20.43 }, { 5.22, 19.24, 20.44 }, { 5.225, 19.23, 20.45 }, { 5.23, 19.21, 20.46 },
                { 5.235, 19.2, 20.47 }, { 5.24, 19.18, 20.48 }, { 5.245, 19.17, 20.49 }, { 5.25, 19.15, 20.5 },
                { 5.255, 19.14, 20.51 }, { 5.26, 19.12, 20.52 }, { 5.265, 19.11, 20.53 }, { 5.27, 19.09, 20.54 },
                { 5.274, 19.08, 20.548 }, { 5.28, 19.06, 20.56 }, { 5.285, 19.05, 20.57 }, { 5.29, 19.03, 20.58 },
                { 5.295, 19.02, 20.59 }, { 5.3, 19.01, 20.6 }, { 5.305, 18.99, 20.61 }, { 5.31, 18.98, 20.62 },
                { 5.314, 18.96, 20.628 }, { 5.32, 18.95, 20.64 }, { 5.325, 18.93, 20.65 }, { 5.33, 18.92, 20.66 },
                { 5.335, 18.9, 20.67 }, { 5.34, 18.89, 20.68 }, { 5.345, 18.88, 20.69 }, { 5.35, 18.86, 20.7 },
                { 5.354, 18.85, 20.708 }, { 5.36, 18.83, 20.72 }, { 5.365, 18.82, 20.73 }, { 5.37, 18.8, 20.74 },
                { 5.375, 18.79, 20.75 }, { 5.38, 18.78, 20.76 }, { 5.385, 18.76, 20.77 }, { 5.39, 18.75, 20.78 },
                { 5.395, 18.73, 20.79 }, { 5.4, 18.72, 20.8 }, { 5.405, 18.7, 20.81 }, { 5.41, 18.69, 20.82 },
                { 5.415, 18.68, 20.83 }, { 5.42, 18.66, 20.84 }, { 5.425, 18.65, 20.85 }, { 5.43, 18.63, 20.86 },
                { 5.435, 18.62, 20.87 }, { 5.44, 18.61, 20.88 }, { 5.445, 18.59, 20.89 }, { 5.45, 18.58, 20.9 },
                { 5.455, 18.57, 20.91 }, { 5.46, 18.55, 20.92 }, { 5.465, 18.54, 20.93 }, { 5.47, 18.52, 20.94 },
                { 5.475, 18.51, 20.95 }, { 5.48, 18.5, 20.96 }, { 5.485, 18.48, 20.97 }, { 5.49, 18.47, 20.98 },
                { 5.495, 18.46, 20.99 }, { 5.5, 18.44, 21.0 }, { 5.505, 18.43, 21.01 }, { 5.51, 18.41, 21.02 },
                { 5.515, 18.4, 21.03 }, { 5.52, 18.39, 21.04 }, { 5.524, 18.38, 21.048 }, { 5.53, 18.36, 21.06 },
                { 5.535, 18.35, 21.07 }, { 5.54, 18.33, 21.08 }, { 5.545, 18.32, 21.09 }, { 5.55, 18.31, 21.1 },
                { 5.555, 18.29, 21.11 }, { 5.56, 18.28, 21.12 }, { 5.564, 18.27, 21.128 }, { 5.57, 18.25, 21.14 },
                { 5.575, 18.24, 21.15 }, { 5.58, 18.23, 21.16 }, { 5.585, 18.21, 21.17 }, { 5.59, 18.2, 21.18 },
                { 5.595, 18.19, 21.19 }, { 5.6, 18.17, 21.2 }, { 5.604, 18.16, 21.208 }, { 5.61, 18.15, 21.22 },
                { 5.615, 18.13, 21.23 }, { 5.62, 18.12, 21.24 }, { 5.625, 18.11, 21.25 }, { 5.63, 18.09, 21.26 },
                { 5.635, 18.08, 21.27 }, { 5.64, 18.07, 21.28 }, { 5.645, 18.06, 21.29 }, { 5.65, 18.04, 21.3 },
                { 5.655, 18.03, 21.31 }, { 5.66, 18.02, 21.32 }, { 5.665, 18.0, 21.33 }, { 5.67, 17.99, 21.34 },
                { 5.675, 17.98, 21.35 }, { 5.68, 17.96, 21.36 }, { 5.685, 17.95, 21.37 }, { 5.69, 17.94, 21.38 },
                { 5.695, 17.93, 21.39 }, { 5.7, 17.91, 21.4 }, { 5.705, 17.9, 21.41 }, { 5.71, 17.89, 21.42 },
                { 5.715, 17.87, 21.43 }, { 5.72, 17.86, 21.44 }, { 5.725, 17.85, 21.45 }, { 5.73, 17.84, 21.46 },
                { 5.735, 17.82, 21.47 }, { 5.74, 17.81, 21.48 }, { 5.745, 17.8, 21.49 }, { 5.75, 17.79, 21.5 },
                { 5.755, 17.77, 21.51 }, { 5.76, 17.76, 21.52 }, { 5.765, 17.75, 21.53 }, { 5.77, 17.74, 21.54 },
                { 5.774, 17.73, 21.548 }, { 5.78, 17.71, 21.56 }, { 5.785, 17.7, 21.57 }, { 5.79, 17.69, 21.58 },
                { 5.795, 17.67, 21.59 }, { 5.8, 17.66, 21.6 }, { 5.805, 17.65, 21.61 }, { 5.81, 17.64, 21.62 },
                { 5.814, 17.63, 21.628 }, { 5.82, 17.61, 21.64 }, { 5.825, 17.6, 21.65 }, { 5.83, 17.59, 21.66 },
                { 5.835, 17.58, 21.67 }, { 5.84, 17.56, 21.68 }, { 5.845, 17.55, 21.69 }, { 5.85, 17.54, 21.7 },
                { 5.854, 17.53, 21.708 }, { 5.86, 17.51, 21.72 }, { 5.865, 17.5, 21.73 }, { 5.87, 17.49, 21.74 },
                { 5.875, 17.48, 21.75 }, { 5.88, 17.47, 21.76 }, { 5.885, 17.45, 21.77 }, { 5.89, 17.44, 21.78 },
                { 5.895, 17.43, 21.79 }, { 5.9, 17.42, 21.8 }, { 5.905, 17.41, 21.81 }, { 5.91, 17.39, 21.82 },
                { 5.915, 17.38, 21.83 }, { 5.92, 17.37, 21.84 }, { 5.925, 17.36, 21.85 }, { 5.93, 17.35, 21.86 },
                { 5.935, 17.33, 21.87 }, { 5.94, 17.32, 21.88 }, { 5.945, 17.31, 21.89 }, { 5.95, 17.3, 21.9 },
                { 5.955, 17.29, 21.91 }, { 5.96, 17.27, 21.92 }, { 5.965, 17.26, 21.93 }, { 5.97, 17.25, 21.94 },
                { 5.975, 17.24, 21.95 }, { 5.98, 17.23, 21.96 }, { 5.985, 17.22, 21.97 }, { 5.99, 17.2, 21.98 },
                { 5.995, 17.19, 21.99 }, { 6.0, 17.18, 22.0 }, { 6.005, 17.17, 22.01 }, { 6.01, 17.16, 22.02 },
                { 6.015, 17.15, 22.03 }, { 6.02, 17.13, 22.04 }, { 6.024, 17.13, 22.048 }, { 6.03, 17.11, 22.06 },
                { 6.035, 17.1, 22.07 }, { 6.04, 17.09, 22.08 }, { 6.045, 17.08, 22.09 }, { 6.05, 17.07, 22.1 },
                { 6.055, 17.05, 22.11 }, { 6.06, 17.04, 22.12 }, { 6.064, 17.03, 22.128 }, { 6.07, 17.02, 22.14 },
                { 6.075, 17.01, 22.15 }, { 6.08, 17.0, 22.16 }, { 6.085, 16.99, 22.17 }, { 6.09, 16.97, 22.18 },
                { 6.095, 16.96, 22.19 }, { 6.1, 16.95, 22.2 }, { 6.104, 16.94, 22.208 }, { 6.11, 16.93, 22.22 },
                { 6.115, 16.92, 22.23 }, { 6.12, 16.91, 22.24 }, { 6.125, 16.9, 22.25 }, { 6.13, 16.88, 22.26 },
                { 6.135, 16.87, 22.27 }, { 6.14, 16.86, 22.28 }, { 6.145, 16.85, 22.29 }, { 6.15, 16.84, 22.3 },
                { 6.155, 16.83, 22.31 }, { 6.16, 16.82, 22.32 }, { 6.165, 16.81, 22.33 }, { 6.17, 16.79, 22.34 },
                { 6.175, 16.78, 22.35 }, { 6.18, 16.77, 22.36 }, { 6.185, 16.76, 22.37 }, { 6.19, 16.75, 22.38 },
                { 6.195, 16.74, 22.39 }, { 6.2, 16.73, 22.4 }, { 6.205, 16.72, 22.41 }, { 6.21, 16.71, 22.42 },
                { 6.215, 16.7, 22.43 }, { 6.22, 16.68, 22.44 }, { 6.225, 16.67, 22.45 }, { 6.23, 16.66, 22.46 },
                { 6.235, 16.65, 22.47 }, { 6.24, 16.64, 22.48 }, { 6.245, 16.63, 22.49 }, { 6.25, 16.62, 22.5 },
                { 6.255, 16.61, 22.51 }, { 6.26, 16.6, 22.52 }, { 6.265, 16.59, 22.53 }, { 6.27, 16.58, 22.54 },
                { 6.274, 16.57, 22.548 }, { 6.28, 16.55, 22.56 }, { 6.285, 16.54, 22.57 }, { 6.29, 16.53, 22.58 },
                { 6.295, 16.52, 22.59 }, { 6.3, 16.51, 22.6 }, { 6.305, 16.5, 22.61 }, { 6.31, 16.49, 22.62 },
                { 6.314, 16.48, 22.628 }, { 6.32, 16.47, 22.64 }, { 6.325, 16.46, 22.65 }, { 6.33, 16.45, 22.66 },
                { 6.335, 16.44, 22.67 }, { 6.34, 16.43, 22.68 }, { 6.345, 16.42, 22.69 }, { 6.35, 16.41, 22.7 },
                { 6.354, 16.4, 22.708 }, { 6.36, 16.39, 22.72 }, { 6.365, 16.37, 22.73 }, { 6.37, 16.36, 22.74 },
                { 6.375, 16.35, 22.75 }, { 6.38, 16.34, 22.76 }, { 6.385, 16.33, 22.77 }, { 6.39, 16.32, 22.78 },
                { 6.395, 16.31, 22.79 }, { 6.4, 16.3, 22.8 }, { 6.405, 16.29, 22.81 }, { 6.41, 16.28, 22.82 },
                { 6.415, 16.27, 22.83 }, { 6.42, 16.26, 22.84 }, { 6.425, 16.25, 22.85 }, { 6.43, 16.24, 22.86 },
                { 6.435, 16.23, 22.87 }, { 6.44, 16.22, 22.88 }, { 6.445, 16.21, 22.89 }, { 6.45, 16.2, 22.9 },
                { 6.455, 16.19, 22.91 }, { 6.46, 16.18, 22.92 }, { 6.465, 16.17, 22.93 }, { 6.47, 16.16, 22.94 },
                { 6.475, 16.15, 22.95 }, { 6.48, 16.14, 22.96 }, { 6.485, 16.13, 22.97 }, { 6.49, 16.12, 22.98 },
                { 6.495, 16.11, 22.99 }, { 6.5, 16.1, 23.0 }, { 6.505, 16.09, 23.01 }, { 6.51, 16.08, 23.02 },
                { 6.515, 16.07, 23.03 }, { 6.52, 16.06, 23.04 }, { 6.524, 16.05, 23.048 }, { 6.53, 16.04, 23.06 },
                { 6.535, 16.03, 23.07 }, { 6.54, 16.02, 23.08 }, { 6.545, 16.01, 23.09 }, { 6.55, 16.0, 23.1 },
                { 6.555, 15.99, 23.11 }, { 6.56, 15.98, 23.12 }, { 6.564, 15.97, 23.128 }, { 6.57, 15.96, 23.14 },
                { 6.575, 15.95, 23.15 }, { 6.58, 15.94, 23.16 }, { 6.585, 15.93, 23.17 }, { 6.59, 15.92, 23.18 },
                { 6.595, 15.91, 23.19 }, { 6.6, 15.9, 23.2 }, { 6.604, 15.89, 23.208 }, { 6.61, 15.88, 23.22 },
                { 6.615, 15.87, 23.23 }, { 6.62, 15.86, 23.24 }, { 6.625, 15.85, 23.25 }, { 6.63, 15.84, 23.26 },
                { 6.635, 15.83, 23.27 }, { 6.64, 15.82, 23.28 }, { 6.645, 15.81, 23.29 }, { 6.65, 15.8, 23.3 },
                { 6.655, 15.79, 23.31 }, { 6.66, 15.78, 23.32 }, { 6.665, 15.77, 23.33 }, { 6.67, 15.76, 23.34 },
                { 6.675, 15.75, 23.35 }, { 6.68, 15.74, 23.36 }, { 6.685, 15.73, 23.37 }, { 6.69, 15.73, 23.38 },
                { 6.695, 15.72, 23.39 }, { 6.7, 15.71, 23.4 }, { 6.705, 15.7, 23.41 }, { 6.71, 15.69, 23.42 },
                { 6.715, 15.68, 23.43 }, { 6.72, 15.67, 23.44 }, { 6.725, 15.66, 23.45 }, { 6.73, 15.65, 23.46 },
                { 6.735, 15.64, 23.47 }, { 6.74, 15.63, 23.48 }, { 6.745, 15.62, 23.49 }, { 6.75, 15.61, 23.5 },
                { 6.755, 15.6, 23.51 }, { 6.76, 15.59, 23.52 }, { 6.765, 15.58, 23.53 }, { 6.77, 15.57, 23.54 },
                { 6.774, 15.57, 23.548 }, { 6.78, 15.56, 23.56 }, { 6.785, 15.55, 23.57 }, { 6.79, 15.54, 23.58 },
                { 6.795, 15.53, 23.59 }, { 6.8, 15.52, 23.6 }, { 6.805, 15.51, 23.61 }, { 6.81, 15.5, 23.62 },
                { 6.814, 15.49, 23.628 }, { 6.82, 15.48, 23.64 }, { 6.825, 15.47, 23.65 }, { 6.83, 15.46, 23.66 },
                { 6.835, 15.45, 23.67 }, { 6.84, 15.44, 23.68 }, { 6.845, 15.44, 23.69 }, { 6.85, 15.43, 23.7 },
                { 6.854, 15.42, 23.708 }, { 6.86, 15.41, 23.72 }, { 6.865, 15.4, 23.73 }, { 6.87, 15.39, 23.74 },
                { 6.875, 15.38, 23.75 }, { 6.88, 15.37, 23.76 }, { 6.885, 15.36, 23.77 }, { 6.89, 15.35, 23.78 },
                { 6.895, 15.34, 23.79 }, { 6.9, 15.34, 23.8 }, { 6.905, 15.33, 23.81 }, { 6.91, 15.32, 23.82 },
                { 6.915, 15.31, 23.83 }, { 6.92, 15.3, 23.84 }, { 6.925, 15.29, 23.85 }, { 6.93, 15.28, 23.86 },
                { 6.935, 15.27, 23.87 }, { 6.94, 15.26, 23.88 }, { 6.945, 15.25, 23.89 }, { 6.95, 15.25, 23.9 },
                { 6.955, 15.24, 23.91 }, { 6.96, 15.23, 23.92 }, { 6.965, 15.22, 23.93 }, { 6.97, 15.21, 23.94 },
                { 6.975, 15.2, 23.95 }, { 6.98, 15.19, 23.96 }, { 6.985, 15.18, 23.97 }, { 6.99, 15.17, 23.98 },
                { 6.995, 15.17, 23.99 }, { 7.0, 15.16, 24.0 }, { 7.005, 15.15, 24.01 }, { 7.01, 15.14, 24.02 },
                { 7.015, 15.13, 24.03 }, { 7.02, 15.12, 24.04 }, { 7.024, 15.12, 24.048 }, { 7.03, 15.1, 24.06 },
                { 7.035, 15.1, 24.07 }, { 7.04, 15.09, 24.08 }, { 7.045, 15.08, 24.09 }, { 7.05, 15.07, 24.1 },
                { 7.055, 15.06, 24.11 }, { 7.06, 15.05, 24.12 }, { 7.064, 15.05, 24.128 }, { 7.07, 15.04, 24.14 },
                { 7.075, 15.03, 24.15 }, { 7.08, 15.02, 24.16 }, { 7.085, 15.01, 24.17 }, { 7.09, 15.0, 24.18 },
                { 7.095, 14.99, 24.19 }, { 7.1, 14.98, 24.2 }, { 7.104, 14.98, 24.208 }, { 7.11, 14.97, 24.22 },
                { 7.115, 14.96, 24.23 }, { 7.12, 14.95, 24.24 }, { 7.125, 14.94, 24.25 }, { 7.13, 14.93, 24.26 },
                { 7.135, 14.92, 24.27 }, { 7.14, 14.92, 24.28 }, { 7.145, 14.91, 24.29 }, { 7.15, 14.9, 24.3 },
                { 7.155, 14.89, 24.31 }, { 7.16, 14.88, 24.32 }, { 7.165, 14.87, 24.33 }, { 7.17, 14.87, 24.34 },
                { 7.175, 14.86, 24.35 }, { 7.18, 14.85, 24.36 }, { 7.185, 14.84, 24.37 }, { 7.19, 14.83, 24.38 },
                { 7.195, 14.82, 24.39 }, { 7.2, 14.81, 24.4 }, { 7.205, 14.81, 24.41 }, { 7.21, 14.8, 24.42 },
                { 7.215, 14.79, 24.43 }, { 7.22, 14.78, 24.44 }, { 7.225, 14.77, 24.45 }, { 7.23, 14.76, 24.46 },
                { 7.235, 14.76, 24.47 }, { 7.24, 14.75, 24.48 }, { 7.245, 14.74, 24.49 }, { 7.25, 14.73, 24.5 },
                { 7.255, 14.72, 24.51 }, { 7.26, 14.72, 24.52 }, { 7.265, 14.71, 24.53 }, { 7.27, 14.7, 24.54 },
                { 7.274, 14.69, 24.548 }, { 7.28, 14.68, 24.56 }, { 7.285, 14.67, 24.57 }, { 7.29, 14.67, 24.58 },
                { 7.295, 14.66, 24.59 }, { 7.3, 14.65, 24.6 }, { 7.305, 14.64, 24.61 }, { 7.31, 14.63, 24.62 },
                { 7.314, 14.63, 24.628 }, { 7.32, 14.62, 24.64 }, { 7.325, 14.61, 24.65 }, { 7.33, 14.6, 24.66 },
                { 7.335, 14.59, 24.67 }, { 7.34, 14.59, 24.68 }, { 7.345, 14.58, 24.69 }, { 7.35, 14.57, 24.7 },
                { 7.354, 14.56, 24.708 }, { 7.36, 14.55, 24.72 }, { 7.365, 14.55, 24.73 }, { 7.37, 14.54, 24.74 },
                { 7.375, 14.53, 24.75 }, { 7.38, 14.52, 24.76 }, { 7.385, 14.51, 24.77 }, { 7.39, 14.51, 24.78 },
                { 7.395, 14.5, 24.79 }, { 7.4, 14.49, 24.8 }, { 7.405, 14.48, 24.81 }, { 7.41, 14.47, 24.82 },
                { 7.415, 14.47, 24.83 }, { 7.42, 14.46, 24.84 }, { 7.425, 14.45, 24.85 }, { 7.43, 14.44, 24.86 },
                { 7.435, 14.43, 24.87 }, { 7.44, 14.43, 24.88 }, { 7.445, 14.42, 24.89 }, { 7.45, 14.41, 24.9 },
                { 7.455, 14.4, 24.91 }, { 7.46, 14.4, 24.92 }, { 7.465, 14.39, 24.93 }, { 7.47, 14.38, 24.94 },
                { 7.475, 14.37, 24.95 }, { 7.48, 14.36, 24.96 }, { 7.485, 14.36, 24.97 }, { 7.49, 14.35, 24.98 },
                { 7.495, 14.34, 24.99 }, { 7.5, 14.33, 25.0 }, { 7.505, 14.33, 25.01 }, { 7.51, 14.32, 25.02 },
                { 7.515, 14.31, 25.03 }, { 7.52, 14.3, 25.04 }, { 7.524, 14.3, 25.048 }, { 7.53, 14.29, 25.06 },
                { 7.535, 14.28, 25.07 }, { 7.54, 14.27, 25.08 }, { 7.545, 14.26, 25.09 }, { 7.55, 14.26, 25.1 },
                { 7.555, 14.25, 25.11 }, { 7.56, 14.24, 25.12 }, { 7.564, 14.23, 25.128 }, { 7.57, 14.23, 25.14 },
                { 7.575, 14.22, 25.15 }, { 7.58, 14.21, 25.16 }, { 7.585, 14.2, 25.17 }, { 7.59, 14.2, 25.18 },
                { 7.595, 14.19, 25.19 }, { 7.6, 14.18, 25.2 }, { 7.604, 14.17, 25.208 }, { 7.61, 14.16, 25.22 },
                { 7.615, 14.16, 25.23 }, { 7.62, 14.15, 25.24 }, { 7.625, 14.14, 25.25 }, { 7.63, 14.13, 25.26 },
                { 7.635, 14.13, 25.27 }, { 7.64, 14.12, 25.28 }, { 7.645, 14.11, 25.29 }, { 7.65, 14.1, 25.3 },
                { 7.655, 14.1, 25.31 }, { 7.66, 14.09, 25.32 }, { 7.665, 14.08, 25.33 }, { 7.67, 14.08, 25.34 },
                { 7.675, 14.07, 25.35 }, { 7.68, 14.06, 25.36 }, { 7.685, 14.05, 25.37 }, { 7.69, 14.05, 25.38 },
                { 7.695, 14.04, 25.39 }, { 7.7, 14.03, 25.4 }, { 7.705, 14.02, 25.41 }, { 7.71, 14.02, 25.42 },
                { 7.715, 14.01, 25.43 }, { 7.72, 14.0, 25.44 }, { 7.725, 13.99, 25.45 }, { 7.73, 13.99, 25.46 },
                { 7.735, 13.98, 25.47 }, { 7.74, 13.97, 25.48 }, { 7.745, 13.96, 25.49 }, { 7.75, 13.96, 25.5 },
                { 7.755, 13.95, 25.51 }, { 7.76, 13.94, 25.52 }, { 7.765, 13.94, 25.53 }, { 7.77, 13.93, 25.54 },
                { 7.774, 13.92, 25.548 }, { 7.78, 13.91, 25.56 }, { 7.785, 13.91, 25.57 }, { 7.79, 13.9, 25.58 },
                { 7.795, 13.89, 25.59 }, { 7.8, 13.89, 25.6 }, { 7.805, 13.88, 25.61 }, { 7.81, 13.87, 25.62 },
                { 7.814, 13.87, 25.628 }, { 7.82, 13.86, 25.64 }, { 7.825, 13.85, 25.65 }, { 7.83, 13.84, 25.66 },
                { 7.835, 13.84, 25.67 }, { 7.84, 13.83, 25.68 }, { 7.845, 13.82, 25.69 }, { 7.85, 13.81, 25.7 },
                { 7.854, 13.81, 25.708 }, { 7.86, 13.8, 25.72 }, { 7.865, 13.79, 25.73 }, { 7.87, 13.79, 25.74 },
                { 7.875, 13.78, 25.75 }, { 7.88, 13.77, 25.76 }, { 7.885, 13.76, 25.77 }, { 7.89, 13.76, 25.78 },
                { 7.895, 13.75, 25.79 }, { 7.9, 13.74, 25.8 }, { 7.905, 13.74, 25.81 }, { 7.91, 13.73, 25.82 },
                { 7.915, 13.72, 25.83 }, { 7.92, 13.72, 25.84 }, { 7.925, 13.71, 25.85 }, { 7.93, 13.7, 25.86 },
                { 7.935, 13.68, 25.87 }, { 7.94, 13.69, 25.88 }, { 7.945, 13.67, 25.89 }, { 7.95, 13.66, 25.9 },
                { 7.955, 13.66, 25.91 }, { 7.96, 13.65, 25.92 }, { 7.965, 13.64, 25.93 }, { 7.97, 13.64, 25.94 },
                { 7.975, 13.63, 25.95 }, { 7.98, 13.62, 25.96 }, { 7.985, 13.62, 25.97 }, { 7.99, 13.61, 25.98 },
                { 7.995, 13.6, 25.99 }, { 8.0, 13.59, 26.0 }, { 8.005, 13.59, 26.01 }, { 8.01, 13.58, 26.02 },
                { 8.015, 13.57, 26.03 }, { 8.02, 13.57, 26.04 }, { 8.025, 13.56, 26.05 }, { 8.03, 13.55, 26.06 },
                { 8.035, 13.55, 26.07 }, { 8.04, 13.54, 26.08 }, { 8.045, 13.53, 26.09 }, { 8.05, 13.53, 26.1 },
                { 8.055, 13.52, 26.11 }, { 8.06, 13.51, 26.12 }, { 8.065, 13.51, 26.13 }, { 8.07, 13.5, 26.14 },
                { 8.075, 13.49, 26.15 }, { 8.08, 13.49, 26.16 }, { 8.085, 13.48, 26.17 }, { 8.09, 13.47, 26.18 },
                { 8.095, 13.47, 26.19 }, { 8.1, 13.46, 26.2 }, { 8.105, 13.45, 26.21 }, { 8.11, 13.45, 26.22 },
                { 8.115, 13.44, 26.23 }, { 8.12, 13.43, 26.24 }, { 8.125, 13.43, 26.25 }, { 8.13, 13.42, 26.26 },
                { 8.135, 13.41, 26.27 }, { 8.14, 13.41, 26.28 }, { 8.145, 13.4, 26.29 }, { 8.15, 13.39, 26.3 },
                { 8.155, 13.39, 26.31 }, { 8.16, 13.38, 26.32 }, { 8.165, 13.37, 26.33 }, { 8.17, 13.37, 26.34 },
                { 8.175, 13.36, 26.35 }, { 8.18, 13.35, 26.36 }, { 8.185, 13.35, 26.37 }, { 8.19, 13.34, 26.38 },
                { 8.195, 13.33, 26.39 }, { 8.2, 13.33, 26.4 }, { 8.205, 13.32, 26.41 }, { 8.21, 13.32, 26.42 },
                { 8.215, 13.31, 26.43 }, { 8.22, 13.3, 26.44 }, { 8.225, 13.3, 26.45 }, { 8.23, 13.29, 26.46 },
                { 8.235, 13.28, 26.47 }, { 8.24, 13.28, 26.48 }, { 8.245, 13.27, 26.49 }, { 8.25, 13.26, 26.5 },
                { 8.255, 13.26, 26.51 }, { 8.26, 13.25, 26.52 }, { 8.265, 13.24, 26.53 }, { 8.27, 13.24, 26.54 },
                { 8.275, 13.23, 26.55 }, { 8.28, 13.23, 26.56 }, { 8.285, 13.22, 26.57 }, { 8.29, 13.21, 26.58 },
                { 8.295, 13.21, 26.59 }, { 8.3, 13.2, 26.6 }, { 8.305, 13.19, 26.61 }, { 8.31, 13.19, 26.62 },
                { 8.315, 13.18, 26.63 }, { 8.32, 13.17, 26.64 }, { 8.325, 13.17, 26.65 }, { 8.33, 13.16, 26.66 },
                { 8.335, 13.16, 26.67 }, { 8.34, 13.15, 26.68 }, { 8.345, 13.14, 26.69 }, { 8.35, 13.14, 26.7 },
                { 8.355, 13.13, 26.71 }, { 8.36, 13.12, 26.72 }, { 8.365, 13.12, 26.73 }, { 8.37, 13.11, 26.74 },
                { 8.375, 13.1, 26.75 }, { 8.38, 13.1, 26.76 }, { 8.385, 13.09, 26.77 }, { 8.39, 13.09, 26.78 },
                { 8.395, 13.08, 26.79 }, { 8.4, 13.07, 26.8 }, { 8.405, 13.07, 26.81 }, { 8.41, 13.06, 26.82 },
                { 8.415, 13.05, 26.83 }, { 8.42, 13.05, 26.84 }, { 8.425, 13.04, 26.85 }, { 8.43, 13.04, 26.86 },
                { 8.435, 13.03, 26.87 }, { 8.44, 13.02, 26.88 }, { 8.445, 13.02, 26.89 }, { 8.45, 13.01, 26.9 },
                { 8.455, 13.01, 26.91 }, { 8.46, 13.0, 26.92 }, { 8.465, 12.99, 26.93 }, { 8.47, 12.99, 26.94 },
                { 8.475, 12.98, 26.95 }, { 8.48, 12.97, 26.96 }, { 8.485, 12.97, 26.97 }, { 8.49, 12.96, 26.98 },
                { 8.495, 12.96, 26.99 }, { 8.5, 12.95, 27.0 }, { 8.505, 12.94, 27.01 }, { 8.51, 12.94, 27.02 },
                { 8.515, 12.93, 27.03 }, { 8.52, 12.93, 27.04 }, { 8.525, 12.92, 27.05 }, { 8.53, 12.91, 27.06 },
                { 8.535, 12.91, 27.07 }, { 8.54, 12.9, 27.08 }, { 8.545, 12.9, 27.09 }, { 8.55, 12.89, 27.1 },
                { 8.555, 12.88, 27.11 }, { 8.56, 12.88, 27.12 }, { 8.565, 12.87, 27.13 }, { 8.57, 12.87, 27.14 },
                { 8.575, 12.86, 27.15 }, { 8.58, 12.85, 27.16 }, { 8.585, 12.85, 27.17 }, { 8.59, 12.84, 27.18 },
                { 8.595, 12.84, 27.19 }, { 8.6, 12.83, 27.2 }, { 8.605, 12.82, 27.21 }, { 8.61, 12.82, 27.22 },
                { 8.615, 12.81, 27.23 }, { 8.62, 12.81, 27.24 }, { 8.625, 12.8, 27.25 }, { 8.63, 12.79, 27.26 },
                { 8.635, 12.79, 27.27 }, { 8.64, 12.78, 27.28 }, { 8.645, 12.78, 27.29 }, { 8.65, 12.77, 27.3 },
                { 8.655, 12.76, 27.31 }, { 8.66, 12.76, 27.32 }, { 8.665, 12.75, 27.33 }, { 8.67, 12.75, 27.34 },
                { 8.675, 12.74, 27.35 }, { 8.68, 12.73, 27.36 }, { 8.685, 12.73, 27.37 }, { 8.69, 12.72, 27.38 },
                { 8.695, 12.72, 27.39 }, { 8.7, 12.71, 27.4 }, { 8.705, 12.71, 27.41 }, { 8.71, 12.7, 27.42 },
                { 8.715, 12.69, 27.43 }, { 8.72, 12.69, 27.44 }, { 8.725, 12.68, 27.45 }, { 8.73, 12.68, 27.46 },
                { 8.735, 12.67, 27.47 }, { 8.74, 12.66, 27.48 }, { 8.745, 12.66, 27.49 }, { 8.75, 12.65, 27.5 },
                { 8.755, 12.65, 27.51 }, { 8.76, 12.64, 27.52 }, { 8.765, 12.64, 27.53 }, { 8.77, 12.63, 27.54 },
                { 8.775, 12.62, 27.55 }, { 8.78, 12.62, 27.56 }, { 8.785, 12.61, 27.57 }, { 8.79, 12.61, 27.58 },
                { 8.795, 12.6, 27.59 }, { 8.8, 12.6, 27.6 }, { 8.805, 12.59, 27.61 }, { 8.81, 12.58, 27.62 },
                { 8.815, 12.58, 27.63 }, { 8.82, 12.57, 27.64 }, { 8.825, 12.57, 27.65 }, { 8.83, 12.56, 27.66 },
                { 8.835, 12.56, 27.67 }, { 8.84, 12.55, 27.68 }, { 8.845, 12.54, 27.69 }, { 8.85, 12.54, 27.7 },
                { 8.855, 12.53, 27.71 }, { 8.86, 12.53, 27.72 }, { 8.865, 12.52, 27.73 }, { 8.87, 12.52, 27.74 },
                { 8.875, 12.51, 27.75 }, { 8.88, 12.51, 27.76 }, { 8.885, 12.5, 27.77 }, { 8.89, 12.49, 27.78 },
                { 8.895, 12.49, 27.79 }, { 8.9, 12.48, 27.8 }, { 8.905, 12.48, 27.81 }, { 8.91, 12.47, 27.82 },
                { 8.915, 12.47, 27.83 }, { 8.92, 12.46, 27.84 }, { 8.925, 12.45, 27.85 }, { 8.93, 12.45, 27.86 },
                { 8.935, 12.44, 27.87 }, { 8.94, 12.44, 27.88 }, { 8.945, 12.43, 27.89 }, { 8.95, 12.43, 27.9 },
                { 8.955, 12.42, 27.91 }, { 8.96, 12.42, 27.92 }, { 8.965, 12.41, 27.93 }, { 8.97, 12.4, 27.94 },
                { 8.975, 12.4, 27.95 }, { 8.98, 12.39, 27.96 }, { 8.985, 12.39, 27.97 }, { 8.99, 12.38, 27.98 },
                { 8.995, 12.38, 27.99 }, { 9.0, 12.37, 28.0 }, { 9.005, 12.37, 28.01 }, { 9.01, 12.36, 28.02 },
                { 9.015, 12.36, 28.03 }, { 9.02, 12.35, 28.04 }, { 9.025, 12.34, 28.05 }, { 9.03, 12.34, 28.06 },
                { 9.035, 12.33, 28.07 }, { 9.04, 12.33, 28.08 }, { 9.045, 12.32, 28.09 }, { 9.05, 12.32, 28.1 },
                { 9.055, 12.31, 28.11 }, { 9.06, 12.31, 28.12 }, { 9.065, 12.3, 28.13 }, { 9.07, 12.3, 28.14 },
                { 9.075, 12.29, 28.15 }, { 9.08, 12.28, 28.16 }, { 9.085, 12.28, 28.17 }, { 9.09, 12.27, 28.18 },
                { 9.095, 12.27, 28.19 }, { 9.1, 12.26, 28.2 }, { 9.105, 12.26, 28.21 }, { 9.11, 12.25, 28.22 },
                { 9.115, 12.25, 28.23 }, { 9.12, 12.24, 28.24 }, { 9.125, 12.24, 28.25 }, { 9.13, 12.23, 28.26 },
                { 9.135, 12.23, 28.27 }, { 9.14, 12.22, 28.28 }, { 9.145, 12.22, 28.29 }, { 9.15, 12.21, 28.3 },
                { 9.155, 12.2, 28.31 }, { 9.16, 12.2, 28.32 }, { 9.165, 12.19, 28.33 }, { 9.17, 12.19, 28.34 },
                { 9.175, 12.18, 28.35 }, { 9.18, 12.18, 28.36 }, { 9.185, 12.17, 28.37 }, { 9.19, 12.17, 28.38 },
                { 9.195, 12.16, 28.39 }, { 9.2, 12.16, 28.4 }, { 9.205, 12.15, 28.41 }, { 9.21, 12.15, 28.42 },
                { 9.215, 12.14, 28.43 }, { 9.22, 12.14, 28.44 }, { 9.225, 12.13, 28.45 }, { 9.23, 12.13, 28.46 },
                { 9.235, 12.12, 28.47 }, { 9.24, 12.11, 28.48 }, { 9.245, 12.11, 28.49 }, { 9.25, 12.1, 28.5 },
                { 9.255, 12.1, 28.51 }, { 9.26, 12.09, 28.52 }, { 9.265, 12.09, 28.53 }, { 9.27, 12.08, 28.54 },
                { 9.275, 12.08, 28.55 }, { 9.28, 12.07, 28.56 }, { 9.285, 12.07, 28.57 }, { 9.29, 12.06, 28.58 },
                { 9.295, 12.06, 28.59 }, { 9.3, 12.05, 28.6 }, { 9.305, 12.05, 28.61 }, { 9.31, 12.04, 28.62 },
                { 9.315, 12.04, 28.63 }, { 9.32, 12.03, 28.64 }, { 9.325, 12.03, 28.65 }, { 9.33, 12.02, 28.66 },
                { 9.335, 12.02, 28.67 }, { 9.34, 12.01, 28.68 }, { 9.345, 12.01, 28.69 }, { 9.35, 12.0, 28.7 },
                { 9.355, 12.0, 28.71 }, { 9.36, 11.99, 28.72 }, { 9.365, 11.99, 28.73 }, { 9.37, 11.98, 28.74 },
                { 9.375, 11.98, 28.75 }, { 9.38, 11.97, 28.76 }, { 9.385, 11.97, 28.77 }, { 9.39, 11.96, 28.78 },
                { 9.395, 11.96, 28.79 }, { 9.4, 11.95, 28.8 }, { 9.405, 11.95, 28.81 }, { 9.41, 11.94, 28.82 },
                { 9.415, 11.94, 28.83 }, { 9.42, 11.93, 28.84 }, { 9.425, 11.93, 28.85 }, { 9.43, 11.92, 28.86 },
                { 9.435, 11.92, 28.87 }, { 9.44, 11.91, 28.88 }, { 9.445, 11.91, 28.89 }, { 9.45, 11.9, 28.9 },
                { 9.455, 11.9, 28.91 }, { 9.46, 11.89, 28.92 }, { 9.465, 11.89, 28.93 }, { 9.47, 11.88, 28.94 },
                { 9.475, 11.88, 28.95 }, { 9.48, 11.87, 28.96 }, { 9.485, 11.87, 28.97 }, { 9.49, 11.86, 28.98 },
                { 9.495, 11.86, 28.99 }, { 9.5, 11.85, 29.0 }, { 9.505, 11.85, 29.01 }, { 9.51, 11.84, 29.02 },
                { 9.515, 11.84, 29.03 }, { 9.52, 11.83, 29.04 }, { 9.525, 11.83, 29.05 }, { 9.53, 11.82, 29.06 },
                { 9.535, 11.82, 29.07 }, { 9.54, 11.81, 29.08 }, { 9.545, 11.81, 29.09 }, { 9.55, 11.8, 29.1 },
                { 9.555, 11.8, 29.11 }, { 9.56, 11.79, 29.12 }, { 9.565, 11.79, 29.13 }, { 9.57, 11.78, 29.14 },
                { 9.575, 11.78, 29.15 }, { 9.58, 11.77, 29.16 }, { 9.585, 11.77, 29.17 }, { 9.59, 11.76, 29.18 },
                { 9.595, 11.76, 29.19 }, { 9.6, 11.75, 29.2 }, { 9.605, 11.75, 29.21 }, { 9.61, 11.74, 29.22 },
                { 9.615, 11.74, 29.23 }, { 9.62, 11.73, 29.24 }, { 9.625, 11.73, 29.25 }, { 9.63, 11.72, 29.26 },
                { 9.635, 11.72, 29.27 }, { 9.64, 11.71, 29.28 }, { 9.645, 11.71, 29.29 }, { 9.65, 11.7, 29.3 },
                { 9.655, 11.7, 29.31 }, { 9.66, 11.69, 29.32 }, { 9.665, 11.69, 29.33 }, { 9.67, 11.68, 29.34 },
                { 9.675, 11.68, 29.35 }, { 9.68, 11.68, 29.36 }, { 9.685, 11.67, 29.37 }, { 9.69, 11.67, 29.38 },
                { 9.695, 11.66, 29.39 }, { 9.7, 11.66, 29.4 }, { 9.705, 11.65, 29.41 }, { 9.71, 11.65, 29.42 },
                { 9.715, 11.64, 29.43 }, { 9.72, 11.64, 29.44 }, { 9.725, 11.63, 29.45 }, { 9.73, 11.63, 29.46 },
                { 9.735, 11.62, 29.47 }, { 9.74, 11.62, 29.48 }, { 9.745, 11.61, 29.49 }, { 9.75, 11.61, 29.5 },
                { 9.755, 11.6, 29.51 }, { 9.76, 11.6, 29.52 }, { 9.765, 11.59, 29.53 }, { 9.77, 11.59, 29.54 },
                { 9.775, 11.59, 29.55 }, { 9.78, 11.58, 29.56 }, { 9.785, 11.58, 29.57 }, { 9.79, 11.57, 29.58 },
                { 9.795, 11.57, 29.59 }, { 9.8, 11.56, 29.6 }, { 9.805, 11.56, 29.61 }, { 9.81, 11.55, 29.62 },
                { 9.815, 11.55, 29.63 }, { 9.82, 11.54, 29.64 }, { 9.825, 11.54, 29.65 }, { 9.83, 11.53, 29.66 },
                { 9.835, 11.53, 29.67 }, { 9.84, 11.52, 29.68 }, { 9.845, 11.52, 29.69 }, { 9.85, 11.52, 29.7 },
                { 9.855, 11.51, 29.71 }, { 9.86, 11.51, 29.72 }, { 9.865, 11.5, 29.73 }, { 9.87, 11.5, 29.74 },
                { 9.875, 11.49, 29.75 }, { 9.88, 11.49, 29.76 }, { 9.885, 11.48, 29.77 }, { 9.89, 11.48, 29.78 },
                { 9.895, 11.47, 29.79 }, { 9.9, 11.47, 29.8 }, { 9.905, 11.46, 29.81 }, { 9.91, 11.46, 29.82 },
                { 9.915, 11.46, 29.83 }, { 9.92, 11.45, 29.84 }, { 9.925, 11.45, 29.85 }, { 9.93, 11.44, 29.86 },
                { 9.935, 11.44, 29.87 }, { 9.94, 11.43, 29.88 }, { 9.945, 11.43, 29.89 }, { 9.95, 11.42, 29.9 },
                { 9.955, 11.42, 29.91 }, { 9.96, 11.41, 29.92 }, { 9.965, 11.41, 29.93 }, { 9.97, 11.41, 29.94 },
                { 9.975, 11.4, 29.95 }, { 9.98, 11.4, 29.96 }, { 9.985, 11.39, 29.97 }, { 9.99, 11.39, 29.98 },
                { 9.995, 11.38, 29.99 }, { 10.0, 11.38, 30.0 }, { 10.005, 11.37, 30.01 }, { 10.01, 11.37, 30.02 },
                { 10.015, 11.36, 30.03 }, { 10.02, 11.36, 30.04 }, { 10.025, 11.36, 30.05 }, { 10.03, 11.35, 30.06 },
                { 10.035, 11.35, 30.07 }, { 10.04, 11.34, 30.08 }, { 10.045, 11.34, 30.09 }, { 10.05, 11.33, 30.1 },
                { 10.055, 11.33, 30.11 }, { 10.06, 11.32, 30.12 }, { 10.065, 11.32, 30.13 }, { 10.07, 11.32, 30.14 },
                { 10.075, 11.31, 30.15 }, { 10.08, 11.31, 30.16 }, { 10.085, 11.3, 30.17 }, { 10.09, 11.3, 30.18 },
                { 10.095, 11.29, 30.19 }, { 10.1, 11.29, 30.2 }, { 10.105, 11.28, 30.21 }, { 10.11, 11.28, 30.22 },
                { 10.115, 11.28, 30.23 }, { 10.12, 11.27, 30.24 }, { 10.125, 11.27, 30.25 }, { 10.13, 11.26, 30.26 },
                { 10.135, 11.26, 30.27 }, { 10.14, 11.25, 30.28 }, { 10.145, 11.25, 30.29 }, { 10.15, 11.25, 30.3 },
                { 10.155, 11.24, 30.31 }, { 10.16, 11.24, 30.32 }, { 10.165, 11.23, 30.33 }, { 10.17, 11.23, 30.34 },
                { 10.175, 11.22, 30.35 }, { 10.18, 11.22, 30.36 }, { 10.185, 11.21, 30.37 }, { 10.19, 11.21, 30.38 },
                { 10.195, 11.21, 30.39 }, { 10.2, 11.2, 30.4 }, { 10.205, 11.2, 30.41 }, { 10.21, 11.19, 30.42 },
                { 10.215, 11.19, 30.43 }, { 10.22, 11.18, 30.44 }, { 10.225, 11.18, 30.45 }, { 10.23, 11.18, 30.46 },
                { 10.235, 11.17, 30.47 }, { 10.24, 11.17, 30.48 }, { 10.245, 11.16, 30.49 }, { 10.25, 11.16, 30.5 },
                { 10.255, 11.15, 30.51 }, { 10.26, 11.15, 30.52 }, { 10.265, 11.15, 30.53 }, { 10.27, 11.14, 30.54 },
                { 10.275, 11.14, 30.55 }, { 10.28, 11.13, 30.56 }, { 10.285, 11.13, 30.57 }, { 10.29, 11.12, 30.58 },
                { 10.295, 11.12, 30.59 }, { 10.3, 11.12, 30.6 }, { 10.305, 11.11, 30.61 }, { 10.31, 11.11, 30.62 },
                { 10.315, 11.1, 30.63 }, { 10.32, 11.1, 30.64 }, { 10.325, 11.09, 30.65 }, { 10.33, 11.09, 30.66 },
                { 10.335, 11.09, 30.67 }, { 10.34, 11.08, 30.68 }, { 10.345, 11.08, 30.69 }, { 10.35, 11.07, 30.7 },
                { 10.355, 11.07, 30.71 }, { 10.36, 11.06, 30.72 }, { 10.365, 11.06, 30.73 }, { 10.37, 11.06, 30.74 },
                { 10.375, 11.05, 30.75 }, { 10.38, 11.05, 30.76 }, { 10.385, 11.04, 30.77 }, { 10.39, 11.04, 30.78 },
                { 10.395, 11.04, 30.79 }, { 10.4, 11.03, 30.8 }, { 10.405, 11.03, 30.81 }, { 10.41, 11.02, 30.82 },
                { 10.415, 11.02, 30.83 }, { 10.42, 11.01, 30.84 }, { 10.425, 11.01, 30.85 }, { 10.43, 11.01, 30.86 },
                { 10.435, 11.0, 30.87 }, { 10.44, 11.0, 30.88 }, { 10.445, 10.99, 30.89 }, { 10.45, 10.99, 30.9 },
                { 10.455, 10.99, 30.91 }, { 10.46, 10.98, 30.92 }, { 10.465, 10.98, 30.93 }, { 10.47, 10.97, 30.94 },
                { 10.475, 10.97, 30.95 }, { 10.48, 10.96, 30.96 }, { 10.485, 10.96, 30.97 }, { 10.49, 10.96, 30.98 },
                { 10.495, 10.95, 30.99 }, { 10.5, 10.95, 31.0 }, { 10.505, 10.94, 31.01 }, { 10.51, 10.94, 31.02 },
                { 10.515, 10.94, 31.03 }, { 10.52, 10.93, 31.04 }, { 10.525, 10.93, 31.05 }, { 10.53, 10.92, 31.06 },
                { 10.535, 10.92, 31.07 }, { 10.54, 10.92, 31.08 }, { 10.545, 10.91, 31.09 }, { 10.55, 10.91, 31.1 },
                { 10.555, 10.9, 31.11 }, { 10.56, 10.9, 31.12 }, { 10.565, 10.9, 31.13 }, { 10.57, 10.89, 31.14 },
                { 10.575, 10.89, 31.15 }, { 10.58, 10.88, 31.16 }, { 10.585, 10.88, 31.17 }, { 10.59, 10.88, 31.18 },
                { 10.595, 10.87, 31.19 }, { 10.6, 10.87, 31.2 }, { 10.605, 10.86, 31.21 }, { 10.61, 10.86, 31.22 },
                { 10.615, 10.86, 31.23 }, { 10.62, 10.85, 31.24 }, { 10.625, 10.85, 31.25 }, { 10.63, 10.84, 31.26 },
                { 10.635, 10.84, 31.27 }, { 10.64, 10.84, 31.28 }, { 10.645, 10.83, 31.29 }, { 10.65, 10.83, 31.3 },
                { 10.655, 10.82, 31.31 }, { 10.66, 10.82, 31.32 }, { 10.665, 10.82, 31.33 }, { 10.67, 10.81, 31.34 },
                { 10.675, 10.81, 31.35 }, { 10.68, 10.8, 31.36 }, { 10.685, 10.8, 31.37 }, { 10.69, 10.8, 31.38 },
                { 10.695, 10.79, 31.39 }, { 10.7, 10.79, 31.4 }, { 10.705, 10.78, 31.41 }, { 10.71, 10.78, 31.42 },
                { 10.715, 10.78, 31.43 }, { 10.72, 10.77, 31.44 }, { 10.725, 10.77, 31.45 }, { 10.73, 10.76, 31.46 },
                { 10.735, 10.76, 31.47 }, { 10.74, 10.76, 31.48 }, { 10.745, 10.75, 31.49 }, { 10.75, 10.75, 31.5 },
                { 10.755, 10.74, 31.51 }, { 10.76, 10.74, 31.52 }, { 10.765, 10.74, 31.53 }, { 10.77, 10.73, 31.54 },
                { 10.775, 10.73, 31.55 }, { 10.78, 10.72, 31.56 }, { 10.785, 10.72, 31.57 }, { 10.79, 10.72, 31.58 },
                { 10.795, 10.71, 31.59 }, { 10.8, 10.71, 31.6 }, { 10.805, 10.71, 31.61 }, { 10.81, 10.7, 31.62 },
                { 10.815, 10.7, 31.63 }, { 10.82, 10.69, 31.64 }, { 10.825, 10.69, 31.65 }, { 10.83, 10.69, 31.66 },
                { 10.835, 10.68, 31.67 }, { 10.84, 10.68, 31.68 }, { 10.845, 10.67, 31.69 }, { 10.85, 10.67, 31.7 },
                { 10.855, 10.67, 31.71 }, { 10.86, 10.66, 31.72 }, { 10.865, 10.66, 31.73 }, { 10.87, 10.66, 31.74 },
                { 10.875, 10.65, 31.75 }, { 10.88, 10.65, 31.76 }, { 10.885, 10.64, 31.77 }, { 10.89, 10.64, 31.78 },
                { 10.895, 10.64, 31.79 }, { 10.9, 10.63, 31.8 }, { 10.905, 10.63, 31.81 }, { 10.91, 10.62, 31.82 },
                { 10.915, 10.62, 31.83 }, { 10.92, 10.62, 31.84 }, { 10.925, 10.61, 31.85 }, { 10.93, 10.61, 31.86 },
                { 10.935, 10.61, 31.87 }, { 10.94, 10.6, 31.88 }, { 10.945, 10.6, 31.89 }, { 10.95, 10.59, 31.9 },
                { 10.955, 10.59, 31.91 }, { 10.96, 10.59, 31.92 }, { 10.965, 10.58, 31.93 }, { 10.97, 10.58, 31.94 },
                { 10.975, 10.58, 31.95 }, { 10.98, 10.57, 31.96 }, { 10.985, 10.57, 31.97 }, { 10.99, 10.56, 31.98 },
                { 10.995, 10.56, 31.99 }, { 11.0, 10.56, 32.0 }, { 11.005, 10.55, 32.01 }, { 11.01, 10.55, 32.02 },
                { 11.015, 10.55, 32.03 }, { 11.02, 10.54, 32.04 }, { 11.025, 10.54, 32.05 }, { 11.03, 10.53, 32.06 },
                { 11.035, 10.53, 32.07 }, { 11.04, 10.53, 32.08 }, { 11.045, 10.52, 32.09 }, { 11.05, 10.52, 32.1 },
                { 11.055, 10.52, 32.11 }, { 11.06, 10.51, 32.12 }, { 11.065, 10.51, 32.13 }, { 11.07, 10.5, 32.14 },
                { 11.075, 10.5, 32.15 }, { 11.08, 10.5, 32.16 }, { 11.085, 10.49, 32.17 }, { 11.09, 10.49, 32.18 },
                { 11.095, 10.49, 32.19 }, { 11.1, 10.48, 32.2 }, { 11.105, 10.48, 32.21 }, { 11.11, 10.47, 32.22 },
                { 11.115, 10.47, 32.23 }, { 11.12, 10.47, 32.24 }, { 11.125, 10.46, 32.25 }, { 11.13, 10.46, 32.26 },
                { 11.135, 10.46, 32.27 }, { 11.14, 10.45, 32.28 }, { 11.145, 10.45, 32.29 }, { 11.15, 10.45, 32.3 },
                { 11.155, 10.44, 32.31 }, { 11.16, 10.44, 32.32 }, { 11.165, 10.43, 32.33 }, { 11.17, 10.43, 32.34 },
                { 11.175, 10.43, 32.35 }, { 11.18, 10.42, 32.36 }, { 11.185, 10.42, 32.37 }, { 11.19, 10.42, 32.38 },
                { 11.195, 10.41, 32.39 }, { 11.2, 10.41, 32.4 }, { 11.205, 10.41, 32.41 }, { 11.21, 10.4, 32.42 },
                { 11.215, 10.4, 32.43 }, { 11.22, 10.39, 32.44 }, { 11.225, 10.39, 32.45 }, { 11.23, 10.39, 32.46 },
                { 11.235, 10.38, 32.47 }, { 11.24, 10.38, 32.48 }, { 11.245, 10.38, 32.49 }, { 11.25, 10.37, 32.5 },
                { 11.255, 10.37, 32.51 }, { 11.26, 10.37, 32.52 }, { 11.265, 10.36, 32.53 }, { 11.27, 10.36, 32.54 },
                { 11.275, 10.36, 32.55 }, { 11.28, 10.35, 32.56 }, { 11.285, 10.35, 32.57 }, { 11.29, 10.34, 32.58 },
                { 11.295, 10.34, 32.59 }, { 11.3, 10.34, 32.6 }, { 11.305, 10.33, 32.61 }, { 11.31, 10.33, 32.62 },
                { 11.315, 10.33, 32.63 }, { 11.32, 10.32, 32.64 }, { 11.325, 10.32, 32.65 }, { 11.33, 10.32, 32.66 },
                { 11.335, 10.31, 32.67 }, { 11.34, 10.31, 32.68 }, { 11.345, 10.31, 32.69 }, { 11.35, 10.3, 32.7 },
                { 11.355, 10.3, 32.71 }, { 11.36, 10.29, 32.72 }, { 11.365, 10.29, 32.73 }, { 11.37, 10.29, 32.74 },
                { 11.375, 10.28, 32.75 }, { 11.38, 10.28, 32.76 }, { 11.385, 10.28, 32.77 }, { 11.39, 10.27, 32.78 },
                { 11.395, 10.27, 32.79 }, { 11.4, 10.27, 32.8 }, { 11.405, 10.26, 32.81 }, { 11.41, 10.26, 32.82 },
                { 11.415, 10.26, 32.83 }, { 11.42, 10.25, 32.84 }, { 11.425, 10.25, 32.85 }, { 11.43, 10.25, 32.86 },
                { 11.435, 10.24, 32.87 }, { 11.44, 10.24, 32.88 }, { 11.445, 10.24, 32.89 }, { 11.45, 10.23, 32.9 },
                { 11.455, 10.23, 32.91 }, { 11.46, 10.23, 32.92 }, { 11.465, 10.22, 32.93 }, { 11.47, 10.22, 32.94 },
                { 11.475, 10.21, 32.95 }, { 11.48, 10.21, 32.96 }, { 11.485, 10.21, 32.97 }, { 11.49, 10.2, 32.98 },
                { 11.495, 10.2, 32.99 }, { 11.5, 10.2, 33.0 }, { 11.505, 10.19, 33.01 }, { 11.51, 10.19, 33.02 },
                { 11.515, 10.19, 33.03 }, { 11.52, 10.18, 33.04 }, { 11.525, 10.18, 33.05 }, { 11.53, 10.18, 33.06 },
                { 11.535, 10.17, 33.07 }, { 11.54, 10.17, 33.08 }, { 11.545, 10.17, 33.09 }, { 11.55, 10.16, 33.1 },
                { 11.555, 10.16, 33.11 }, { 11.56, 10.16, 33.12 }, { 11.565, 10.15, 33.13 }, { 11.57, 10.15, 33.14 },
                { 11.575, 10.15, 33.15 }, { 11.58, 10.14, 33.16 }, { 11.585, 10.14, 33.17 }, { 11.59, 10.14, 33.18 },
                { 11.595, 10.13, 33.19 }, { 11.6, 10.13, 33.2 }, { 11.605, 10.13, 33.21 }, { 11.61, 10.12, 33.22 },
                { 11.615, 10.12, 33.23 }, { 11.62, 10.12, 33.24 }, { 11.625, 10.11, 33.25 }, { 11.63, 10.11, 33.26 },
                { 11.635, 10.11, 33.27 }, { 11.64, 10.1, 33.28 }, { 11.645, 10.1, 33.29 }, { 11.65, 10.1, 33.3 },
                { 11.655, 10.09, 33.31 }, { 11.66, 10.09, 33.32 }, { 11.665, 10.09, 33.33 }, { 11.67, 10.08, 33.34 },
                { 11.675, 10.08, 33.35 }, { 11.68, 10.08, 33.36 }, { 11.685, 10.07, 33.37 }, { 11.69, 10.07, 33.38 },
                { 11.695, 10.07, 33.39 }, { 11.7, 10.06, 33.4 }, { 11.705, 10.06, 33.41 }, { 11.71, 10.06, 33.42 },
                { 11.715, 10.05, 33.43 }, { 11.72, 10.05, 33.44 }, { 11.725, 10.05, 33.45 }, { 11.73, 10.04, 33.46 },
                { 11.735, 10.04, 33.47 }, { 11.74, 10.04, 33.48 }, { 11.745, 10.03, 33.49 }, { 11.75, 10.03, 33.5 },
                { 11.755, 10.03, 33.51 }, { 11.76, 10.02, 33.52 }, { 11.765, 10.02, 33.53 }, { 11.77, 10.02, 33.54 },
                { 11.775, 10.01, 33.55 }, { 11.78, 10.01, 33.56 }, { 11.785, 10.01, 33.57 }, { 11.79, 10.0, 33.58 },
                { 11.795, 10.0, 33.59 }, { 11.8, 10.0, 33.6 }, { 11.805, 9.99, 33.61 }, { 11.81, 9.99, 33.62 },
                { 11.815, 9.99, 33.63 }, { 11.82, 9.98, 33.64 }, { 11.825, 9.98, 33.65 }, { 11.83, 9.98, 33.66 },
                { 11.835, 9.97, 33.67 }, { 11.84, 9.97, 33.68 }, { 11.845, 9.97, 33.69 }, { 11.85, 9.96, 33.7 },
                { 11.855, 9.96, 33.71 }, { 11.86, 9.96, 33.72 }, { 11.865, 9.95, 33.73 }, { 11.87, 9.95, 33.74 },
                { 11.875, 9.95, 33.75 }, { 11.88, 9.94, 33.76 }, { 11.885, 9.94, 33.77 }, { 11.89, 9.94, 33.78 },
                { 11.895, 9.93, 33.79 }, { 11.9, 9.93, 33.8 }, { 11.905, 9.93, 33.81 }, { 11.91, 9.92, 33.82 },
                { 11.915, 9.92, 33.83 }, { 11.92, 9.92, 33.84 }, { 11.925, 9.92, 33.85 }, { 11.93, 9.91, 33.86 },
                { 11.935, 9.91, 33.87 }, { 11.94, 9.91, 33.88 }, { 11.945, 9.9, 33.89 }, { 11.95, 9.9, 33.9 },
                { 11.955, 9.9, 33.91 }, { 11.96, 9.89, 33.92 }, { 11.965, 9.89, 33.93 }, { 11.97, 9.89, 33.94 },
                { 11.975, 9.88, 33.95 }, { 11.98, 9.88, 33.96 }, { 11.985, 9.88, 33.97 }, { 11.99, 9.87, 33.98 },
                { 11.995, 9.87, 33.99 }, { 12.0, 9.87, 34.0 }, { 12.005, 9.86, 34.01 }, { 12.01, 9.86, 34.02 },
                { 12.015, 9.86, 34.03 }, { 12.02, 9.85, 34.04 }, { 12.025, 9.85, 34.05 }, { 12.03, 9.85, 34.06 },
                { 12.035, 9.85, 34.07 }, { 12.04, 9.84, 34.08 }, { 12.045, 9.84, 34.09 }, { 12.05, 9.84, 34.1 },
                { 12.055, 9.83, 34.11 }, { 12.06, 9.83, 34.12 }, { 12.065, 9.83, 34.13 }, { 12.07, 9.82, 34.14 },
                { 12.075, 9.82, 34.15 }, { 12.08, 9.82, 34.16 }, { 12.085, 9.81, 34.17 }, { 12.09, 9.81, 34.18 },
                { 12.095, 9.81, 34.19 }, { 12.1, 9.8, 34.2 }, { 12.105, 9.8, 34.21 }, { 12.11, 9.8, 34.22 },
                { 12.115, 9.8, 34.23 }, { 12.12, 9.79, 34.24 }, { 12.125, 9.79, 34.25 }, { 12.13, 9.79, 34.26 },
                { 12.135, 9.78, 34.27 }, { 12.14, 9.78, 34.28 }, { 12.145, 9.78, 34.29 }, { 12.15, 9.77, 34.3 },
                { 12.155, 9.77, 34.31 }, { 12.16, 9.77, 34.32 }, { 12.165, 9.76, 34.33 }, { 12.17, 9.76, 34.34 },
                { 12.175, 9.76, 34.35 }, { 12.18, 9.76, 34.36 }, { 12.185, 9.75, 34.37 }, { 12.19, 9.75, 34.38 },
                { 12.195, 9.75, 34.39 }, { 12.2, 9.74, 34.4 }, { 12.205, 9.74, 34.41 }, { 12.21, 9.74, 34.42 },
                { 12.215, 9.73, 34.43 }, { 12.22, 9.73, 34.44 }, { 12.225, 9.73, 34.45 }, { 12.23, 9.72, 34.46 },
                { 12.235, 9.72, 34.47 }, { 12.24, 9.72, 34.48 }, { 12.245, 9.72, 34.49 }, { 12.25, 9.71, 34.5 },
                { 12.255, 9.71, 34.51 }, { 12.26, 9.71, 34.52 }, { 12.265, 9.7, 34.53 }, { 12.27, 9.7, 34.54 },
                { 12.275, 9.7, 34.55 }, { 12.28, 9.69, 34.56 }, { 12.285, 9.69, 34.57 }, { 12.29, 9.69, 34.58 },
                { 12.295, 9.69, 34.59 }, { 12.3, 9.68, 34.6 }, { 12.305, 9.68, 34.61 }, { 12.31, 9.68, 34.62 },
                { 12.315, 9.67, 34.63 }, { 12.32, 9.67, 34.64 }, { 12.325, 9.67, 34.65 }, { 12.33, 9.66, 34.66 },
                { 12.335, 9.66, 34.67 }, { 12.34, 9.66, 34.68 }, { 12.345, 9.66, 34.69 }, { 12.35, 9.65, 34.7 },
                { 12.355, 9.65, 34.71 }, { 12.36, 9.65, 34.72 }, { 12.365, 9.64, 34.73 }, { 12.37, 9.64, 34.74 },
                { 12.375, 9.64, 34.75 }, { 12.38, 9.63, 34.76 }, { 12.385, 9.63, 34.77 }, { 12.39, 9.63, 34.78 },
                { 12.395, 9.63, 34.79 }, { 12.4, 9.62, 34.8 }, { 12.405, 9.62, 34.81 }, { 12.41, 9.62, 34.82 },
                { 12.415, 9.61, 34.83 }, { 12.42, 9.61, 34.84 }, { 12.425, 9.61, 34.85 }, { 12.43, 9.6, 34.86 },
                { 12.435, 9.6, 34.87 }, { 12.44, 9.6, 34.88 }, { 12.445, 9.6, 34.89 }, { 12.45, 9.59, 34.9 },
                { 12.455, 9.59, 34.91 }, { 12.46, 9.59, 34.92 }, { 12.465, 9.58, 34.93 }, { 12.47, 9.58, 34.94 },
                { 12.475, 9.58, 34.95 }, { 12.48, 9.58, 34.96 }, { 12.485, 9.57, 34.97 }, { 12.49, 9.57, 34.98 },
                { 12.495, 9.57, 34.99 }, { 12.5, 9.56, 35.0 }, { 12.505, 9.56, 35.01 }, { 12.51, 9.56, 35.02 },
                { 12.515, 9.56, 35.03 }, { 12.52, 9.55, 35.04 }, { 12.525, 9.55, 35.05 }, { 12.53, 9.55, 35.06 },
                { 12.535, 9.54, 35.07 }, { 12.54, 9.54, 35.08 }, { 12.545, 9.54, 35.09 }, { 12.55, 9.53, 35.1 },
                { 12.555, 9.53, 35.11 }, { 12.56, 9.53, 35.12 }, { 12.565, 9.53, 35.13 }, { 12.57, 9.52, 35.14 },
                { 12.575, 9.52, 35.15 }, { 12.58, 9.52, 35.16 }, { 12.585, 9.51, 35.17 }, { 12.59, 9.51, 35.18 },
                { 12.595, 9.51, 35.19 }, { 12.6, 9.51, 35.2 }, { 12.605, 9.5, 35.21 }, { 12.61, 9.5, 35.22 },
                { 12.615, 9.5, 35.23 }, { 12.62, 9.49, 35.24 }, { 12.625, 9.49, 35.25 }, { 12.63, 9.49, 35.26 },
                { 12.635, 9.49, 35.27 }, { 12.64, 9.48, 35.28 }, { 12.645, 9.48, 35.29 }, { 12.65, 9.48, 35.3 },
                { 12.655, 9.47, 35.31 }, { 12.66, 9.47, 35.32 }, { 12.665, 9.47, 35.33 }, { 12.67, 9.47, 35.34 },
                { 12.675, 9.46, 35.35 }, { 12.68, 9.46, 35.36 }, { 12.685, 9.46, 35.37 }, { 12.69, 9.45, 35.38 },
                { 12.695, 9.45, 35.39 }, { 12.7, 9.45, 35.4 }, { 12.705, 9.45, 35.41 }, { 12.71, 9.44, 35.42 },
                { 12.715, 9.44, 35.43 }, { 12.72, 9.44, 35.44 }, { 12.725, 9.43, 35.45 }, { 12.73, 9.43, 35.46 },
                { 12.735, 9.43, 35.47 }, { 12.74, 9.43, 35.48 }, { 12.745, 9.42, 35.49 } };
    }

}
