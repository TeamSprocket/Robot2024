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

        public static final double kHorizontalAngle = 66.0;

        public static final double kMaxShooterPivotOutput = 0.3;

        // public static final double kMaxVelocityDeg = 0.0;
        // public static final double kMaxAccelerationDeg = 0.0;

        public static final double kTargetAngleStowed = 5.0;
        public static final double kTargetAngleIntake = 5.0;
        // public static final double kTargetAngleSpeaker = 0.0;
        // public static final double kTargetAngleSpeakerHigh = 0.0;
        public static final double kTargetAngleSpeakerFromAmp = kTargetAngleStowed;
        public static final double kTargetAngleSpeakerFromPodium = kTargetAngleStowed;
        public static final double kTargetAngleSpeakerFromSubwoofer = 10.0;
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

        public static final double kHasNoteCurrentThreshold = 60.0;

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

    public static final class Shintake {
        // Timestuff
        public static final double kDurationScoreAmpSec = 1.0;

        public static final boolean kIsShooterTopInverted = true;
        public static final boolean kIsShooterBottomInverted = true;
        public static final boolean kIsIndexerInverted = true;

        public static final double kMotorShintakeTolerance = 0.025;

        public static final double kIndexerSpeedIntake = 0.3;
        public static final double kIndexerSpeedScoreAmp = -0.6;
        public static final double kIndexerEjectNoteSpeed = -0.5;
        public static final double kIndexerHasNoteIndexThreshold = 10000;

        // JUST IN CASE

        public static final double kShooterGearRatio = 0.6666666666;
        public static final double kIndexerGearRatio = 2.0;

        public static final double kShooterWheelDiameter = Conversions.inchesToMeters(2.0);

        public static final double kShooterIncramentMultiplier = 0.01;

        public static final double kHasNoteCurrentThreshold = 60.0;

        //public static final double kAtGoalTolerance = 0.1; // 0.05 = precise for dynamic

        public static final double kPivotAngleStowed = 10; // 20
        public static final double kPivotAngleIntake = 60;
        public static final double kScoreAmp = 60;
        public static final double kRollSpeedStowed = 0.0;
        public static final double kRollSpeedIntake = 0.6;
        public static final double kRollSpeedScoreAmp = -0.7;
        public static final double kEjectNoteSpeed = -0.6;

        public static final double kMaxPivotOutput = 0.1;

        public static final double kPivotIntakeGearRatio = 36.0;

        public static final double kPivotAngleOffset = 0;

        public static final double kPPivot = 0.002914; // 1, 0.1
        public static final double kIPivot = 0.0;
        public static final double kDPivot = 0.00002445;
        public static final PIDConst kPIDPivot = new PIDConst(kPPivot, kIPivot, kDPivot);
    }

    public static final class Limelight {
        public static final double kAcceptableVolatilityThreshold = 0.2;

        public static final double kMaxDrivingSpeed = 0.0;
        public static final double kMaxTurningSpeed = 0.0;

        public static final int kVolatilitySlidingWindowLen = 20;

        public static final double kPlimelight = 0.003;
        public static final double kDlimelight = 0.0;
        public static final double kIlimelight = 0.0;

        // should go into robotmap
        public static final double kLimelightMountAngleDegrees = 5.0;
        public static final double kGoalHeightMeters = 1.45;
        public static final double kLimelightHeightMeters = 0.53;
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
        public static double kPTurnMotorFL = 0.0025; // 0.0125
        public static double kITurnMotorFL = 0.00;
        public static double kDTurnMotorFL = 0.000; // 0.000026

        public static double kPTurnMotorFR = 0.0025; // 0.0125
        public static double kITurnMotorFR = 0.00;
        public static double kDTurnMotorFR = 0.000; // 0.000026

        public static double kPTurnMotorBL = 0.0025; // 0.0125
        public static double kITurnMotorBL = 0.00;
        public static double kDTurnMotorBL = 0.000; // 0.000026

        public static double kPTurnMotorBR = 0.0025; // 0.0125
        public static double kITurnMotorBR = 0.00;
        public static double kDTurnMotorBR = 0.000; // 0.000026
        

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

        public static double kMaxSpeed = 1.1; //1.1 // 0.8 //1.0 // 0.6 - latest
        public static double kMaxAccel = 1.6; //1.6 // 0.7 //1.0 // 1.0 - latest


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

        public static final double kTurnError = 10.0;
        public static final double kFF = 1.0;

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
        
        public static double kCANCoderOffsetFrontLeft = 304.0; // 50.3
        public static double kCANCoderOffsetFrontRight = 20.2; // 126.8
        public static double kCANCoderOffsetBackLeft = 245.3; // 270.2
        public static double kCANCoderOffsetBackRight = 354.9; // 161.6


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
        public static final double[][] vals = {{0.5, 64.07, 7.688}, {0.51, 63.64, 7.701}, {0.52, 87.32, 7.715}, {0.53, 62.78, 7.729}, {0.54, 62.35, 7.742}, {0.55, 61.93, 7.756}, {0.56, 61.51, 7.77}, {0.57, 61.1, 7.784}, {0.58, 60.69, 7.798}, {0.59, 60.29, 7.811}, {0.6, 59.89, 7.825}, {0.61, 59.49, 7.839}, {0.62, 59.1, 7.853}, {0.63, 58.71, 7.866}, {0.64, 58.33, 7.88}, {0.65, 57.95, 7.894}, {0.66, 57.57, 7.907}, {0.67, 57.2, 7.921}, {0.68, 56.83, 7.935}, {0.69, 56.46, 7.949}, {0.7, 56.1, 7.963}, {0.71, 55.75, 7.976}, {0.72, 55.39, 7.99}, {0.73, 55.04, 8.004}, {0.74, 54.7, 8.018}, {0.75, 54.36, 8.031}, {0.76, 54.02, 8.045}, {0.77, 53.68, 8.059}, {0.78, 53.35, 8.072}, {0.79, 53.03, 8.086}, {0.8, 52.7, 8.1}, {0.81, 52.38, 8.114}, {0.82, 52.07, 8.127}, {0.83, 51.75, 8.141}, {0.84, 51.44, 8.155}, {0.85, 51.14, 8.169}, {0.86, 50.84, 8.182}, {0.87, 50.54, 8.196}, {0.88, 50.24, 8.21}, {0.89, 49.95, 8.224}, {0.9, 49.66, 8.238}, {0.91, 49.37, 8.251}, {0.92, 49.09, 8.265}, {0.93, 48.81, 8.279}, {0.94, 48.53, 8.293}, {0.95, 48.26, 8.306}, {0.96, 47.99, 8.32}, {0.97, 47.72, 8.334}, {0.98, 47.46, 8.348}, {0.99, 47.19, 8.361}, {1.0, 46.94, 8.375}, {1.01, 46.68, 8.389}, {1.02, 46.43, 8.402}, {1.03, 46.18, 8.416}, {1.04, 45.93, 8.43}, {1.05, 45.68, 8.444}, {1.06, 45.44, 8.457}, {1.07, 45.2, 8.471}, {1.08, 44.97, 8.485}, {1.09, 44.73, 8.499}, {1.1, 44.5, 8.512}, {1.11, 44.27, 8.526}, {1.12, 44.04, 8.54}, {1.13, 43.82, 8.554}, {1.14, 43.6, 8.567}, {1.15, 43.38, 8.581}, {1.16, 43.16, 8.595}, {1.17, 42.95, 8.609}, {1.18, 42.74, 8.623}, {1.19, 42.53, 8.636}, {1.2, 42.32, 8.65}, {1.21, 42.11, 8.664}, {1.22, 41.91, 8.678}, {1.23, 41.71, 8.691}, {1.24, 41.51, 8.705}, {1.25, 41.31, 8.719}, {1.26, 41.12, 8.732}, {1.27, 40.93, 8.746}, {1.28, 40.74, 8.76}, {1.29, 40.55, 8.774}, {1.3, 40.36, 8.787}, {1.31, 40.18, 8.801}, {1.32, 39.99, 8.815}, {1.33, 39.81, 8.829}, {1.34, 39.63, 8.842}, {1.35, 39.46, 8.856}, {1.36, 39.28, 8.87}, {1.37, 39.11, 8.884}, {1.38, 38.94, 8.898}, {1.39, 38.77, 8.911}, {1.4, 38.6, 8.925}, {1.41, 38.43, 8.939}, {1.42, 38.27, 8.953}, {1.43, 38.1, 8.966}, {1.44, 37.94, 8.98}, {1.45, 37.78, 8.994}, {1.46, 37.62, 9.008}, {1.47, 37.47, 9.021}, {1.48, 37.31, 9.035}, {1.49, 37.16, 9.049}, {1.5, 37.01, 9.062}, {1.51, 36.86, 9.076}, {1.52, 36.71, 9.09}, {1.53, 36.56, 9.104}, {1.54, 36.42, 9.117}, {1.55, 36.27, 9.131}, {1.56, 36.13, 9.145}, {1.57, 35.99, 9.159}, {1.58, 35.85, 9.172}, {1.59, 35.71, 9.186}, {1.6, 35.57, 9.2}, {1.61, 35.43, 9.214}, {1.62, 35.3, 9.227}, {1.63, 35.16, 9.241}, {1.64, 35.03, 9.255}, {1.65, 34.9, 9.269}, {1.66, 34.77, 9.282}, {1.67, 34.64, 9.296}, {1.68, 34.51, 9.31}, {1.69, 34.39, 9.324}, {1.7, 34.26, 9.338}, {1.71, 34.14, 9.351}, {1.72, 34.01, 9.365}, {1.73, 33.89, 9.379}, {1.74, 33.77, 9.393}, {1.75, 33.65, 9.406}, {1.76, 33.53, 9.42}, {1.77, 33.41, 9.434}, {1.78, 33.3, 9.447}, {1.79, 33.18, 9.461}, {1.8, 33.07, 9.475}, {1.81, 32.96, 9.489}, {1.82, 32.84, 9.502}, {1.83, 32.73, 9.516}, {1.84, 32.62, 9.53}, {1.85, 32.51, 9.544}, {1.86, 32.4, 9.558}, {1.87, 32.3, 9.571}, {1.88, 32.19, 9.585}, {1.89, 32.08, 9.599}, {1.9, 31.98, 9.613}, {1.91, 31.87, 9.626}, {1.92, 31.77, 9.64}, {1.93, 31.67, 9.654}, {1.94, 31.57, 9.668}, {1.95, 31.47, 9.681}, {1.96, 31.37, 9.695}, {1.97, 31.27, 9.709}, {1.98, 31.17, 9.723}, {1.99, 31.07, 9.736}, {2.0, 30.98, 9.75}, {2.01, 30.88, 9.764}, {2.02, 30.79, 9.777}, {2.03, 30.69, 9.791}, {2.04, 30.6, 9.805}, {2.05, 30.51, 9.819}, {2.06, 30.42, 9.832}, {2.07, 30.33, 9.846}, {2.08, 30.24, 9.86}, {2.09, 30.15, 9.874}, {2.1, 30.06, 9.887}, {2.11, 29.97, 9.901}, {2.12, 29.88, 9.915}, {2.13, 29.8, 9.929}, {2.14, 29.71, 9.943}, {2.15, 29.62, 9.956}, {2.16, 29.54, 9.97}, {2.17, 29.46, 9.984}, {2.18, 29.37, 9.998}, {2.19, 29.29, 10.011}, {2.2, 29.21, 10.025}, {2.21, 29.13, 10.039}, {2.22, 29.05, 10.053}, {2.23, 28.97, 10.066}, {2.24, 28.89, 10.08}, {2.25, 28.81, 10.094}, {2.26, 28.73, 10.107}, {2.27, 28.65, 10.121}, {2.28, 28.57, 10.135}, {2.29, 83.39, 10.149}, {2.3, 28.42, 10.162}, {2.31, 28.35, 10.176}, {2.32, 28.27, 10.19}, {2.33, 28.2, 10.204}, {2.34, 28.12, 10.217}, {2.35, 28.05, 10.231}, {2.36, 27.98, 10.245}, {2.37, 27.9, 10.259}, {2.38, 27.83, 10.273}, {2.39, 27.76, 10.286}, {2.4, 27.69, 10.3}, {2.41, 27.62, 10.314}, {2.42, 27.55, 10.328}, {2.43, 27.48, 10.341}, {2.44, 27.41, 10.355}, {2.45, 27.35, 10.369}, {2.46, 27.28, 10.383}, {2.47, 27.21, 10.396}, {2.48, 83.21, 10.41}, {2.49, 27.08, 10.424}, {2.5, 27.01, 10.438}, {2.51, 26.95, 10.451}, {2.52, 26.88, 10.465}, {2.53, 26.82, 10.479}, {2.54, 26.75, 10.492}, {2.55, 26.69, 10.506}, {2.56, 26.62, 10.52}, {2.57, 26.56, 10.534}, {2.58, 26.5, 10.547}, {2.59, 26.44, 10.561}, {2.6, 26.38, 10.575}, {2.61, 26.31, 10.589}, {2.62, 26.25, 10.602}, {2.63, 26.19, 10.616}, {2.64, 26.13, 10.63}, {2.65, 26.07, 10.644}, {2.66, 26.01, 10.658}, {2.67, 25.96, 10.671}, {2.68, 25.9, 10.685}, {2.69, 25.84, 10.699}, {2.7, 25.78, 10.713}, {2.71, 25.72, 10.726}, {2.72, 25.67, 10.74}, {2.73, 25.61, 10.754}, {2.74, 25.55, 10.768}, {2.75, 25.5, 10.781}, {2.76, 25.44, 10.795}, {2.77, 25.39, 10.809}, {2.78, 25.33, 10.822}, {2.79, 25.28, 10.836}, {2.8, 25.22, 10.85}, {2.81, 25.17, 10.864}, {2.82, 25.12, 10.877}, {2.83, 25.06, 10.891}, {2.84, 25.01, 10.905}, {2.85, 24.96, 10.919}, {2.86, 24.91, 10.932}, {2.87, 24.85, 10.946}, {2.88, 24.8, 10.96}, {2.89, 24.75, 10.974}, {2.9, 24.7, 10.988}, {2.91, 24.65, 11.001}, {2.92, 24.6, 11.015}, {2.93, 24.55, 11.029}, {2.94, 24.5, 11.043}, {2.95, 24.45, 11.056}, {2.96, 24.4, 11.07}, {2.97, 24.35, 11.084}, {2.98, 24.3, 11.098}, {2.99, 24.25, 11.111}, {3.0, 24.21, 11.125}, {3.01, 24.16, 11.139}, {3.02, 24.11, 11.152}, {3.03, 24.06, 11.166}, {3.04, 24.02, 11.18}, {3.05, 23.97, 11.194}, {3.06, 23.92, 11.207}, {3.07, 23.88, 11.221}, {3.08, 23.83, 11.235}, {3.09, 23.79, 11.249}, {3.1, 23.74, 11.262}, {3.11, 23.69, 11.276}, {3.12, 82.78, 11.29}, {3.13, 23.6, 11.304}, {3.14, 23.56, 11.317}, {3.15, 23.52, 11.331}, {3.16, 23.47, 11.345}, {3.17, 23.43, 11.359}, {3.18, 23.38, 11.373}, {3.19, 23.34, 11.386}, {3.2, 23.3, 11.4}, {3.21, 23.26, 11.414}, {3.22, 23.21, 11.428}, {3.23, 23.17, 11.441}, {3.24, 23.13, 11.455}, {3.25, 23.09, 11.469}, {3.26, 23.04, 11.482}, {3.27, 23.0, 11.496}, {3.28, 22.96, 11.51}, {3.29, 22.92, 11.524}, {3.3, 22.88, 11.537}, {3.31, 22.84, 11.551}, {3.32, 82.69, 11.565}, {3.33, 22.76, 11.579}, {3.34, 22.72, 11.592}, {3.35, 22.68, 11.606}, {3.36, 22.64, 11.62}, {3.37, 22.6, 11.634}, {3.38, 22.56, 11.648}, {3.39, 22.52, 11.661}, {3.4, 22.48, 11.675}, {3.41, 22.44, 11.689}, {3.42, 22.41, 11.703}, {3.43, 22.37, 11.716}, {3.44, 22.33, 11.73}, {3.45, 22.29, 11.744}, {3.46, 22.25, 11.758}, {3.47, 22.22, 11.771}, {3.48, 22.18, 11.785}, {3.49, 22.14, 11.799}, {3.5, 22.1, 11.812}, {3.51, 82.62, 11.826}, {3.52, 22.03, 11.84}, {3.53, 21.99, 11.854}, {3.54, 21.96, 11.867}, {3.55, 21.92, 11.881}, {3.56, 21.89, 11.895}, {3.57, 21.85, 11.909}, {3.58, 21.81, 11.922}, {3.59, 21.78, 11.936}, {3.6, 21.74, 11.95}, {3.61, 21.71, 11.964}, {3.62, 21.67, 11.977}, {3.63, 21.64, 11.991}, {3.64, 82.58, 12.005}, {3.65, 21.57, 12.019}, {3.66, 21.54, 12.033}, {3.67, 21.5, 12.046}, {3.68, 21.47, 12.06}, {3.69, 21.43, 12.074}, {3.7, 21.4, 12.088}, {3.71, 21.37, 12.101}, {3.72, 21.33, 12.115}, {3.73, 21.3, 12.129}, {3.74, 21.27, 12.143}, {3.75, 21.23, 12.156}, {3.76, 21.2, 12.17}, {3.77, 21.17, 12.184}, {3.78, 21.14, 12.197}, {3.79, 21.1, 12.211}, {3.8, 21.07, 12.225}, {3.81, 21.04, 12.239}, {3.82, 21.01, 12.252}, {3.83, 20.98, 12.266}, {3.84, 20.94, 12.28}, {3.85, 20.91, 12.294}, {3.86, 20.88, 12.308}, {3.87, 20.85, 12.321}, {3.88, 20.82, 12.335}, {3.89, 20.79, 12.349}, {3.9, 20.76, 12.363}, {3.91, 20.73, 12.376}, {3.92, 20.7, 12.39}, {3.93, 20.66, 12.404}, {3.94, 20.63, 12.418}, {3.95, 20.6, 12.431}, {3.96, 20.57, 12.445}, {3.97, 20.54, 12.459}, {3.98, 20.51, 12.473}, {3.99, 20.48, 12.486}, {4.0, 20.46, 12.5}, {4.01, 20.43, 12.514}, {4.02, 20.4, 12.527}, {4.03, 20.37, 12.541}, {4.04, 20.34, 12.555}, {4.05, 20.31, 12.569}, {4.06, 20.28, 12.582}, {4.07, 20.25, 12.596}, {4.08, 20.22, 12.61}, {4.09, 20.19, 12.624}, {4.1, 20.17, 12.637}, {4.11, 20.14, 12.651}, {4.12, 20.11, 12.665}, {4.13, 20.08, 12.679}, {4.14, 20.05, 12.692}, {4.15, 20.03, 12.706}, {4.16, 20.0, 12.72}, {4.17, 19.97, 12.734}, {4.18, 19.94, 12.747}, {4.19, 19.92, 12.761}, {4.2, 19.89, 12.775}, {4.21, 19.86, 12.789}, {4.22, 19.83, 12.802}, {4.23, 19.81, 12.816}, {4.24, 19.78, 12.83}, {4.25, 19.75, 12.844}, {4.26, 19.73, 12.857}, {4.27, 19.7, 12.871}, {4.28, 19.67, 12.885}, {4.29, 19.65, 12.899}, {4.3, 19.62, 12.912}, {4.31, 19.59, 12.926}, {4.32, 19.57, 12.94}, {4.33, 19.54, 12.954}, {4.34, 19.52, 12.967}, {4.35, 19.49, 12.981}, {4.36, 19.46, 12.995}, {4.37, 19.44, 13.009}, {4.38, 19.41, 13.023}, {4.39, 19.39, 13.036}, {4.4, 19.36, 13.05}, {4.41, 19.34, 13.064}, {4.42, 19.31, 13.078}, {4.43, 19.29, 13.091}, {4.44, 19.26, 13.105}, {4.45, 19.24, 13.119}, {4.46, 19.21, 13.133}, {4.47, 19.19, 13.146}, {4.48, 19.16, 13.16}, {4.49, 19.14, 13.174}, {4.5, 19.11, 13.188}, {4.51, 19.09, 13.201}, {4.52, 19.07, 13.215}, {4.53, 19.04, 13.229}, {4.54, 19.02, 13.242}, {4.55, 18.99, 13.256}, {4.56, 18.97, 13.27}, {4.57, 18.95, 13.284}, {4.58, 18.92, 13.297}, {4.59, 18.9, 13.311}, {4.6, 18.87, 13.325}, {4.61, 18.85, 13.339}, {4.62, 18.83, 13.352}, {4.63, 18.8, 13.366}, {4.64, 18.78, 13.38}, {4.65, 18.76, 13.394}, {4.66, 18.73, 13.408}, {4.67, 18.71, 13.421}, {4.68, 18.69, 13.435}, {4.69, 18.67, 13.449}, {4.7, 18.64, 13.463}, {4.71, 18.62, 13.476}, {4.72, 18.6, 13.49}, {4.73, 18.58, 13.504}, {4.74, 18.55, 13.518}, {4.75, 18.53, 13.531}, {4.76, 18.51, 13.545}, {4.77, 18.49, 13.559}, {4.78, 18.46, 13.573}, {4.79, 18.44, 13.586}, {4.8, 18.42, 13.6}, {4.81, 18.4, 13.614}, {4.82, 18.38, 13.628}, {4.83, 18.35, 13.641}, {4.84, 18.33, 13.655}, {4.85, 18.31, 13.669}, {4.86, 18.29, 13.683}, {4.87, 18.27, 13.696}, {4.88, 18.25, 13.71}, {4.89, 18.22, 13.724}, {4.9, 18.2, 13.738}, {4.91, 18.18, 13.751}, {4.92, 18.16, 13.765}, {4.93, 18.14, 13.779}, {4.94, 18.12, 13.793}, {4.95, 18.1, 13.806}, {4.96, 18.08, 13.82}, {4.97, 18.06, 13.834}, {4.98, 18.03, 13.848}, {4.99, 18.01, 13.861}, {5.0, 17.99, 13.875}, {5.01, 17.97, 13.889}, {5.02, 17.95, 13.902}, {5.03, 17.93, 13.916}, {5.04, 17.91, 13.93}, {5.05, 17.89, 13.944}, {5.06, 17.87, 13.957}, {5.07, 17.85, 13.971}, {5.08, 17.83, 13.985}, {5.09, 17.81, 13.999}, {5.1, 17.79, 14.012}, {5.11, 17.77, 14.026}, {5.12, 17.75, 14.04}, {5.13, 17.73, 14.054}, {5.14, 17.71, 14.067}, {5.15, 17.69, 14.081}, {5.16, 17.67, 14.095}, {5.17, 17.65, 14.109}, {5.18, 17.63, 14.122}, {5.19, 17.61, 14.136}, {5.2, 17.59, 14.15}, {5.21, 17.57, 14.164}, {5.22, 17.55, 14.177}, {5.23, 17.53, 14.191}, {5.24, 17.52, 14.205}, {5.25, 17.5, 14.219}, {5.26, 17.48, 14.232}, {5.27, 17.46, 14.246}, {5.28, 17.44, 14.26}, {5.29, 17.42, 14.274}, {5.3, 17.4, 14.287}, {5.31, 17.38, 14.301}, {5.32, 17.36, 14.315}, {5.33, 17.34, 14.329}, {5.34, 17.33, 14.342}, {5.35, 17.31, 14.356}, {5.36, 17.29, 14.37}, {5.37, 17.27, 14.384}, {5.38, 17.25, 14.398}, {5.39, 17.23, 14.411}, {5.4, 17.22, 14.425}, {5.41, 17.2, 14.439}, {5.42, 17.18, 14.453}, {5.43, 17.16, 14.466}, {5.44, 17.14, 14.48}, {5.45, 17.12, 14.494}, {5.46, 17.11, 14.508}, {5.47, 17.09, 14.521}, {5.48, 17.07, 14.535}, {5.49, 17.05, 14.549}, {5.5, 17.03, 14.562}, {5.51, 17.02, 14.576}, {5.52, 17.0, 14.59}, {5.53, 16.98, 14.604}, {5.54, 16.96, 14.617}, {5.55, 16.95, 14.631}, {5.56, 16.93, 14.645}, {5.57, 16.91, 14.659}, {5.58, 16.89, 14.672}, {5.59, 16.88, 14.686}, {5.6, 16.86, 14.7}, {5.61, 16.84, 14.714}, {5.62, 16.82, 14.727}, {5.63, 16.81, 14.741}, {5.64, 16.79, 14.755}, {5.65, 16.77, 14.769}, {5.66, 16.76, 14.783}, {5.67, 16.74, 14.796}, {5.68, 16.72, 14.81}, {5.69, 16.7, 14.824}, {5.7, 16.69, 14.838}, {5.71, 16.67, 14.851}, {5.72, 16.65, 14.865}, {5.73, 16.64, 14.879}, {5.74, 16.62, 14.893}, {5.75, 16.6, 14.906}, {5.76, 16.59, 14.92}, {5.77, 16.57, 14.934}, {5.78, 16.55, 14.948}, {5.79, 16.54, 14.961}, {5.8, 16.52, 14.975}, {5.81, 16.5, 14.989}, {5.82, 16.49, 15.003}, {5.83, 16.47, 15.016}, {5.84, 16.45, 15.03}, {5.85, 16.44, 15.044}, {5.86, 82.5, 15.058}, {5.87, 16.41, 15.071}, {5.88, 16.39, 15.085}, {5.89, 16.37, 15.099}, {5.9, 16.36, 15.113}, {5.91, 16.34, 15.126}, {5.92, 16.33, 15.14}, {5.93, 16.31, 15.154}, {5.94, 82.51, 15.168}, {5.95, 16.28, 15.181}, {5.96, 16.26, 15.195}, {5.97, 16.25, 15.209}, {5.98, 16.23, 15.223}, {5.99, 16.22, 15.236}, {6.0, 16.2, 15.25}, {6.01, 16.18, 15.264}, {6.02, 16.17, 15.277}, {6.03, 16.15, 15.291}, {6.04, 16.14, 15.305}, {6.05, 16.12, 15.319}, {6.06, 16.11, 15.332}, {6.07, 16.09, 15.346}, {6.08, 16.08, 15.36}, {6.09, 16.06, 15.374}, {6.1, 16.05, 15.387}, {6.11, 16.03, 15.401}, {6.12, 16.01, 15.415}, {6.13, 16.0, 15.429}, {6.14, 15.98, 15.442}, {6.15, 15.97, 15.456}, {6.16, 15.95, 15.47}, {6.17, 15.94, 15.484}, {6.18, 15.92, 15.497}, {6.19, 15.91, 15.511}, {6.2, 15.89, 15.525}, {6.21, 15.88, 15.539}, {6.22, 15.86, 15.553}, {6.23, 15.85, 15.566}, {6.24, 15.83, 15.58}, {6.25, 15.82, 15.594}, {6.26, 15.81, 15.607}, {6.27, 15.79, 15.621}, {6.28, 15.78, 15.635}, {6.29, 15.76, 15.649}, {6.3, 15.75, 15.662}, {6.31, 15.73, 15.676}, {6.32, 15.72, 15.69}, {6.33, 15.7, 15.704}, {6.34, 15.69, 15.717}, {6.35, 15.67, 15.731}, {6.36, 15.66, 15.745}, {6.37, 15.65, 15.759}, {6.38, 15.63, 15.772}, {6.39, 15.62, 15.786}, {6.4, 15.6, 15.8}, {6.41, 15.59, 15.814}, {6.42, 15.57, 15.828}, {6.43, 15.56, 15.841}, {6.44, 15.55, 15.855}, {6.45, 15.53, 15.869}, {6.46, 15.52, 15.883}, {6.47, 15.5, 15.896}, {6.48, 15.49, 15.91}, {6.49, 15.48, 15.924}, {6.5, 15.46, 15.938}, {6.51, 15.45, 15.951}, {6.52, 15.43, 15.965}, {6.53, 15.42, 15.979}, {6.54, 15.41, 15.992}, {6.55, 15.39, 16.006}, {6.56, 15.38, 16.02}, {6.57, 15.37, 16.034}, {6.58, 15.35, 16.047}, {6.59, 15.34, 16.061}, {6.6, 15.33, 16.075}, {6.61, 15.31, 16.089}, {6.62, 15.3, 16.102}, {6.63, 15.28, 16.116}, {6.64, 15.27, 16.13}, {6.65, 15.26, 16.144}, {6.66, 15.24, 16.157}, {6.67, 15.23, 16.171}, {6.68, 15.22, 16.185}, {6.69, 15.2, 16.199}, {6.7, 15.19, 16.212}, {6.71, 15.18, 16.226}, {6.72, 15.16, 16.24}, {6.73, 15.15, 16.254}, {6.74, 15.14, 16.267}, {6.75, 15.12, 16.281}, {6.76, 15.11, 16.295}, {6.77, 15.1, 16.309}, {6.78, 15.09, 16.322}, {6.79, 15.07, 16.336}, {6.8, 15.06, 16.35}, {6.81, 15.05, 16.364}, {6.82, 15.03, 16.378}, {6.83, 15.02, 16.391}, {6.84, 15.01, 16.405}, {6.85, 14.99, 16.419}, {6.86, 14.98, 16.433}, {6.87, 14.97, 16.446}, {6.88, 14.96, 16.46}, {6.89, 14.94, 16.474}, {6.9, 14.93, 16.488}, {6.91, 14.92, 16.501}, {6.92, 14.9, 16.515}, {6.93, 14.89, 16.529}, {6.94, 14.88, 16.543}, {6.95, 14.87, 16.556}, {6.96, 14.85, 16.57}, {6.97, 14.84, 16.584}, {6.98, 14.83, 16.598}, {6.99, 14.82, 16.611}, {7.0, 14.8, 16.625}, {7.01, 14.79, 16.639}, {7.02, 14.78, 16.652}, {7.03, 14.77, 16.666}, {7.04, 14.75, 16.68}, {7.05, 14.74, 16.694}, {7.06, 14.73, 16.707}, {7.07, 14.72, 16.721}, {7.08, 14.71, 16.735}, {7.09, 14.69, 16.749}, {7.1, 14.68, 16.762}, {7.11, 14.67, 16.776}, {7.12, 14.66, 16.79}, {7.13, 14.64, 16.804}, {7.14, 14.63, 16.817}, {7.15, 14.62, 16.831}, {7.16, 14.61, 16.845}, {7.17, 14.6, 16.859}, {7.18, 14.58, 16.872}, {7.19, 14.57, 16.886}, {7.2, 14.56, 16.9}, {7.21, 14.55, 16.914}, {7.22, 14.54, 16.928}, {7.23, 14.52, 16.941}, {7.24, 14.51, 16.955}, {7.25, 14.5, 16.969}, {7.26, 14.49, 16.983}, {7.27, 14.48, 16.996}, {7.28, 14.46, 17.01}, {7.29, 14.45, 17.024}, {7.3, 14.44, 17.038}, {7.31, 14.43, 17.051}, {7.32, 14.42, 17.065}, {7.33, 14.41, 17.079}, {7.34, 82.75, 17.093}, {7.35, 14.38, 17.106}, {7.36, 14.37, 17.12}, {7.37, 14.36, 17.134}, {7.38, 14.35, 17.148}, {7.39, 14.34, 17.161}, {7.4, 14.32, 17.175}, {7.41, 14.31, 17.189}, {7.42, 14.3, 17.203}, {7.43, 14.29, 17.216}, {7.44, 14.28, 17.23}, {7.45, 14.27, 17.244}, {7.46, 14.26, 17.258}, {7.47, 14.24, 17.271}, {7.48, 14.23, 17.285}, {7.49, 14.22, 17.299}, {7.5, 14.21, 17.312}, {7.51, 14.2, 17.326}, {7.52, 14.19, 17.34}, {7.53, 14.18, 17.354}, {7.54, 14.17, 17.367}, {7.55, 14.15, 17.381}, {7.56, 14.14, 17.395}, {7.57, 14.13, 17.409}, {7.58, 14.12, 17.422}, {7.59, 14.11, 17.436}, {7.6, 14.1, 17.45}, {7.61, 14.09, 17.464}, {7.62, 14.08, 17.477}, {7.63, 14.07, 17.491}, {7.64, 14.05, 17.505}, {7.65, 14.04, 17.519}, {7.66, 14.03, 17.532}, {7.67, 14.02, 17.546}, {7.68, 14.01, 17.56}, {7.69, 14.0, 17.574}, {7.7, 13.99, 17.587}, {7.71, 13.98, 17.601}, {7.72, 13.97, 17.615}, {7.73, 13.96, 17.629}, {7.74, 13.95, 17.642}, {7.75, 13.93, 17.656}, {7.76, 13.92, 17.67}, {7.77, 13.91, 17.684}, {7.78, 13.9, 17.697}, {7.79, 13.89, 17.711}, {7.8, 13.88, 17.725}, {7.81, 13.87, 17.739}, {7.82, 13.86, 17.753}, {7.83, 13.85, 17.766}, {7.84, 13.84, 17.78}, {7.85, 13.83, 17.794}, {7.86, 13.82, 17.808}, {7.87, 13.81, 17.821}, {7.88, 13.8, 17.835}, {7.89, 13.79, 17.849}, {7.9, 13.78, 17.863}, {7.91, 13.76, 17.876}, {7.92, 13.75, 17.89}, {7.93, 13.74, 17.904}, {7.94, 13.73, 17.918}, {7.95, 13.72, 17.931}, {7.96, 13.71, 17.945}, {7.97, 13.7, 17.959}, {7.98, 13.69, 17.973}, {7.99, 13.68, 17.986}, {8.0, 13.67, 18.0}, {8.01, 13.66, 18.014}, {8.02, 13.65, 18.027}, {8.03, 13.64, 18.041}, {8.04, 13.63, 18.055}, {8.05, 13.62, 18.069}, {8.06, 13.61, 18.083}, {8.07, 13.6, 18.096}, {8.08, 13.59, 18.11}, {8.09, 13.58, 18.124}, {8.1, 13.57, 18.137}, {8.11, 13.56, 18.151}, {8.12, 13.55, 18.165}, {8.13, 13.54, 18.179}, {8.14, 13.53, 18.193}, {8.15, 13.52, 18.206}, {8.16, 13.51, 18.22}, {8.17, 13.5, 18.234}, {8.18, 13.49, 18.247}, {8.19, 13.48, 18.261}, {8.2, 13.47, 18.275}, {8.21, 13.46, 18.289}, {8.22, 13.45, 18.303}, {8.23, 13.44, 18.316}, {8.24, 13.43, 18.33}, {8.25, 13.42, 18.344}, {8.26, 13.41, 18.358}, {8.27, 13.4, 18.371}, {8.28, 13.39, 18.385}, {8.29, 13.38, 18.399}, {8.3, 13.37, 18.413}, {8.31, 13.36, 18.426}, {8.32, 13.35, 18.44}, {8.33, 13.34, 18.454}, {8.34, 13.33, 18.468}, {8.35, 13.32, 18.481}, {8.36, 13.31, 18.495}, {8.37, 13.3, 18.509}, {8.38, 13.29, 18.523}, {8.39, 13.28, 18.536}, {8.4, 13.27, 18.55}, {8.41, 82.98, 18.564}, {8.42, 13.25, 18.578}, {8.43, 13.24, 18.591}, {8.44, 13.23, 18.605}, {8.45, 13.22, 18.619}, {8.46, 13.22, 18.633}, {8.47, 13.21, 18.646}, {8.48, 13.2, 18.66}, {8.49, 13.19, 18.674}, {8.5, 13.18, 18.688}, {8.51, 13.17, 18.701}, {8.52, 13.16, 18.715}, {8.53, 13.15, 18.729}, {8.54, 13.14, 18.742}, {8.55, 13.13, 18.756}, {8.56, 13.12, 18.77}, {8.57, 13.11, 18.784}, {8.58, 13.1, 18.797}, {8.59, 13.09, 18.811}, {8.6, 13.08, 18.825}, {8.61, 13.07, 18.839}, {8.62, 13.06, 18.852}, {8.63, 13.06, 18.866}, {8.64, 13.05, 18.88}, {8.65, 13.04, 18.894}, {8.66, 13.03, 18.907}, {8.67, 13.02, 18.921}, {8.68, 13.01, 18.935}, {8.69, 13.0, 18.949}, {8.7, 12.99, 18.962}, {8.71, 12.98, 18.976}, {8.72, 83.05, 18.99}, {8.73, 12.96, 19.004}, {8.74, 12.95, 19.017}, {8.75, 12.95, 19.031}, {8.76, 12.94, 19.045}, {8.77, 12.93, 19.059}, {8.78, 12.92, 19.072}, {8.79, 12.91, 19.086}, {8.8, 12.9, 19.1}, {8.81, 12.89, 19.114}, {8.82, 12.88, 19.128}, {8.83, 12.87, 19.141}, {8.84, 12.86, 19.155}, {8.85, 12.86, 19.169}, {8.86, 12.85, 19.182}, {8.87, 12.84, 19.196}, {8.88, 12.83, 19.21}, {8.89, 12.82, 19.224}, {8.9, 12.81, 19.238}, {8.91, 12.8, 19.251}, {8.92, 12.79, 19.265}, {8.93, 12.78, 19.279}, {8.94, 12.78, 19.292}, {8.95, 12.77, 19.306}, {8.96, 12.76, 19.32}, {8.97, 12.75, 19.334}, {8.98, 12.74, 19.348}, {8.99, 12.73, 19.361}, {9.0, 12.72, 19.375}, {9.01, 12.71, 19.389}, {9.02, 12.71, 19.402}, {9.03, 12.7, 19.416}, {9.04, 12.69, 19.43}, {9.05, 12.68, 19.444}, {9.06, 12.67, 19.458}, {9.07, 12.66, 19.471}, {9.08, 12.65, 19.485}, {9.09, 12.64, 19.499}, {9.1, 12.64, 19.512}, {9.11, 12.63, 19.526}, {9.12, 12.62, 19.54}, {9.13, 12.61, 19.554}, {9.14, 12.6, 19.568}, {9.15, 12.59, 19.581}, {9.16, 12.58, 19.595}, {9.17, 12.58, 19.609}, {9.18, 12.57, 19.622}, {9.19, 12.56, 19.636}, {9.2, 12.55, 19.65}, {9.21, 12.54, 19.664}, {9.22, 12.53, 19.678}, {9.23, 12.53, 19.691}, {9.24, 12.52, 19.705}, {9.25, 12.51, 19.719}, {9.26, 12.5, 19.733}, {9.27, 12.49, 19.746}, {9.28, 12.48, 19.76}, {9.29, 12.48, 19.774}, {9.3, 12.47, 19.788}, {9.31, 12.46, 19.801}, {9.32, 12.45, 19.815}, {9.33, 12.44, 19.829}, {9.34, 12.43, 19.843}, {9.35, 12.43, 19.856}, {9.36, 12.42, 19.87}, {9.37, 12.41, 19.884}, {9.38, 12.4, 19.898}, {9.39, 12.39, 19.911}, {9.4, 12.38, 19.925}, {9.41, 12.38, 19.939}, {9.42, 12.37, 19.953}, {9.43, 12.36, 19.966}, {9.44, 12.35, 19.98}, {9.45, 12.34, 19.994}, {9.46, 12.33, 20.008}, {9.47, 12.33, 20.021}, {9.48, 12.32, 20.035}, {9.49, 12.31, 20.049}, {9.5, 12.3, 20.062}, {9.51, 12.29, 20.076}, {9.52, 12.29, 20.09}, {9.53, 12.28, 20.104}, {9.54, 12.27, 20.117}, {9.55, 12.26, 20.131}, {9.56, 12.25, 20.145}, {9.57, 12.25, 20.159}, {9.58, 12.24, 20.172}, {9.59, 12.23, 20.186}, {9.6, 12.22, 20.2}, {9.61, 12.21, 20.214}, {9.62, 12.21, 20.227}, {9.63, 12.2, 20.241}, {9.64, 12.19, 20.255}, {9.65, 12.18, 20.269}, {9.66, 12.17, 20.282}, {9.67, 12.17, 20.296}, {9.68, 12.16, 20.31}, {9.69, 12.15, 20.324}, {9.7, 12.14, 20.337}, {9.71, 12.14, 20.351}, {9.72, 12.13, 20.365}, {9.73, 12.12, 20.379}, {9.74, 12.11, 20.392}, {9.75, 12.1, 20.406}, {9.76, 12.1, 20.42}, {9.77, 12.09, 20.434}, {9.78, 12.08, 20.447}, {9.79, 12.07, 20.461}, {9.8, 12.06, 20.475}, {9.81, 12.06, 20.489}, {9.82, 12.05, 20.503}, {9.83, 12.04, 20.516}, {9.84, 12.03, 20.53}, {9.85, 12.03, 20.544}, {9.86, 12.02, 20.557}, {9.87, 12.01, 20.571}, {9.88, 12.0, 20.585}, {9.89, 12.0, 20.599}, {9.9, 11.99, 20.613}, {9.91, 11.98, 20.626}, {9.92, 11.97, 20.64}, {9.93, 11.97, 20.654}, {9.94, 11.96, 20.667}, {9.95, 11.95, 20.681}, {9.96, 11.94, 20.695}, {9.97, 11.93, 20.709}, {9.98, 11.93, 20.723}, {9.99, 11.92, 20.736}, {10.0, 11.91, 20.75}};
    }

}
