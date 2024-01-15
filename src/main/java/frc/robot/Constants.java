package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static enum RobotState {
        TELEOP,
        AUTON,
        DISABLED
    }

    // Global
    public static RobotState robotState = RobotState.DISABLED;

    public static final class Claw {}

    public static final class Drivetrain {
        // Measurements
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kModuleOffsetMeters = 0.572;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics (
            new Translation2d(-kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
            new Translation2d(-kModuleOffsetMeters / 2, kModuleOffsetMeters / 2), 
            new Translation2d(kModuleOffsetMeters / 2, -kModuleOffsetMeters / 2),
            new Translation2d(kModuleOffsetMeters / 2, kModuleOffsetMeters / 2)
        );

        
        // PID
        public static final double kPTurnMotor = 0.0125; //0.0125
        public static final double kITurnMotor = 0.0000;
        public static final double kDTurnMotor = 0.0003; //0.0003

        public static final double kPHeading = 1.5978; //0.6
        public static final double kIHeading = 0.0000;
        public static final double kDHeading = 0.12136; 

        public static final double kPTranslationPP = 0.0; 
        public static final double kITranslationPP = 0.0;
        public static final double kDTranslationPP = 0.0; 

        public static final double kPRotationPP = 0.0; 
        public static final double kIRotationPP = 0.0;
        public static final double kDRotationPP = 0.0;

        public static final double kLimelightAlignP = 0.0075;
        public static final double kLimelightAlignI = 0.0;
        public static final double kLimelightAlignD = 0.00015;


        // Speed/Accel 
        public static final double kMaxDriveModuleSpeedMPS = 4.0;

        public static double kMaxSpeed = 0.8; //0.2 
        public static double kMaxAccel = 0.7; 
        
        public static double kMaxTurnSpeed = 0.08; 
        public static double kMaxTurnAccel = 10; // Instant manual turning


        // Misc
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPTranslationPP, kITranslationPP, kDTranslationPP), 
            new PIDConstants(kPRotationPP, kIRotationPP, kDRotationPP),
            kMaxDriveModuleSpeedMPS, 
            kModuleOffsetMeters, 
            new ReplanningConfig()
        );

        public static final boolean kIsFieldOriented = true;

        public static boolean isPrecise = false;
        public static final double kPreciseMultiplier = 0.25;

        public static final double kTurnCurrentLimit = 100;
        public static final double kDriveCurrentLimit = 100;

        // public static boolean CAN_DIRECTION_SWITCH = false;

        public static final boolean FRONT_LEFT_D_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_D_IS_REVERSED = false;
        public static final boolean BACK_LEFT_D_IS_REVERSED = true;
        public static final boolean BACK_RIGHT_D_IS_REVERSED = true;

        // public static final boolean BACK_RIGHT_T_IS_REVERSED = false;
        // public static final boolean FRONT_RIGHT_T_IS_REVERSED = false;
        // public static final boolean BACK_LEFT_T_IS_REVERSED = false;
        // public static final boolean FRONT_LEFT_T_IS_REVERSED = false; 

        public static double kCANCoderOffsetFrontLeft = -53.7;
        public static double kCANCoderOffsetFrontRight = -331.4;
        public static double kCANCoderOffsetBackLeft = 131.1;
        public static double kCANCoderOffsetBackRight = 234.1;

        // public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(303.2);
        // public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(305.5);
        // public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(44.6);
        // public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(77.5);

    }

    public static final class Auton { 
        public static final boolean kFacingDriversOnStart = true;

        // Balance
        public static final double kChargeStationAngle = 14.0; // TODO: tune (pitch angle of bot on charge station)
        public static final double kOnChargeStationTolerance = 1.0; // TODO: tune (pitch angle tolerance for bot to be considered on charge station)
        public static final double kChargeStationBalanceTolerance = 1.0; // TODO: tune (pitch angle tolerance while climbing for charge station to be considered falling)
        public static final double BALANCE_END_TIME_THRESHOLD = 0.5; // 0.5
        public static final double BALANCE_END_ANGLE_THRESHOLD = 5; 
        public static final double kSpeedWhileClimbing = 0.02;

        // PID Turn
        public static final double kPTurn = 0.3; //0.2
        public static final double kITurn = 0;
        public static final double kDTurn = 0.0015; //0.001

        // Auton Parser
        public static final double[][] ROUTINE = {{0.0, 0.0, 0.0, 0.0, 2.356194490192345, -2.356194490192345, 0.7853981633974483, -0.7853981633974483}, {0.0, 0.0, 0.0, 0.0, 2.356194490192345, -2.356194490192345, 0.7853981633974483, -0.7853981633974483}, {0.01632753567810988, 0.01632753567810988, 0.01632753567810988, 0.01632753567810988, 0.5992400782190582, 0.5992400782190582, 0.5992400782190582, 0.5992400782190582}, {0.045247694500382775, 0.045247694500382775, 0.045247694500382775, 0.045247694500382775, 0.6846573169729425, 0.6846573169729425, 0.6846573169729425, 0.6846573169729425}, {0.06266177140471169, 0.06266177140471169, 0.06266177140471169, 0.06266177140471169, 0.5091910067230891, 0.5091910067230891, 0.5091910067230891, 0.5091910067230891}, {0.08016732334164174, 0.08016732334164174, 0.08016732334164174, 0.08016732334164174, 0.32434342321250337, 0.32434342321250337, 0.32434342321250337, 0.32434342321250337}, {0.091932882326037, 0.091932882326037, 0.091932882326037, 0.091932882326037, 0.10671194840788394, 0.10671194840788394, 0.10671194840788394, 0.10671194840788394}, {0.11471118539925268, 0.11471118539925268, 0.11471118539925268, 0.11471118539925268, -0.12221485846378785, -0.12221485846378785, -0.12221485846378785, -0.12221485846378785}, {0.13380261262755627, 0.13380261262755627, 0.13380261262755627, 0.13380261262755627, -0.1101202332569859, -0.1101202332569859, -0.1101202332569859, -0.1101202332569859}, {0.15311012579147015, 0.15311012579147015, 0.15311012579147015, 0.15311012579147015, 0.02857223334470323, 0.02857223334470323, 0.02857223334470323, 0.02857223334470323}, {0.17262679999999905, 0.17262679999999905, 0.17262679999999905, 0.17262679999999905, -0.026589199911371875, -0.026589199911371875, -0.026589199911371875, -0.026589199911371875}, {0.1696616057362413, 0.1696616057362413, 0.1696616057362413, 0.1696616057362413, 0.09192166589838945, 0.09192166589838945, 0.09192166589838945, 0.09192166589838945}, {0.14946914366316486, 0.14946914366316486, 0.14946914366316486, 0.14946914366316486, 0.07649652311010587, 0.07649652311010587, 0.07649652311010587, 0.07649652311010587}, {0.12854159999999834, 0.12854159999999834, 0.12854159999999834, 0.12854159999999834, -0.017950274699994665, -0.017950274699994665, -0.017950274699994665, -0.017950274699994665}, {0.1086057999999994, 0.1086057999999994, 0.1086057999999994, 0.1086057999999994, -0.01494161707287134, -0.01494161707287134, -0.01494161707287134, -0.01494161707287134}, {0.08996351522967358, 0.08996351522967358, 0.08996351522967358, 0.08996351522967358, -0.1946322226946833, -0.1946322226946833, -0.1946322226946833, -0.1946322226946833}, {0.07725821375413663, 0.07725821375413663, 0.07725821375413663, 0.07725821375413663, -0.49731800029766904, -0.49731800029766904, -0.49731800029766904, -0.49731800029766904}, {0.05093995690025627, 0.05093995690025627, 0.05093995690025627, 0.05093995690025627, -0.32790321829266794, -0.32790321829266794, -0.32790321829266794, -0.32790321829266794}, {0.028568599999998417, 0.028568599999998417, 0.028568599999998417, 0.028568599999998417, -0.010528990596116563, -0.010528990596116563, -0.010528990596116563, -0.010528990596116563}, {0.03446327558488935, 0.03446327558488935, 0.03446327558488935, 0.03446327558488935, 0.35335570156577917, 0.35335570156577917, 0.35335570156577917, 0.35335570156577917}, {0.039735920082465286, 0.039735920082465286, 0.039735920082465286, 0.039735920082465286, 0.41649042051391627, 0.41649042051391627, 0.41649042051391627, 0.41649042051391627}, {0.016072400000000188, 0.016072400000000188, 0.016072400000000188, 0.016072400000000188, 0.007900858168476077, 0.007900858168476077, 0.007900858168476077, 0.007900858168476077}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}};
        
        // PID Drive
        // public static final double kPDriveToTargetXY = 1;
        // public static final double kIDriveToTargetXY = 0.0;
        // public static final double kDDriveToTargetXY = 0.0;

        // public static final double kPDriveToTargetT = 1;
        // public static final double kIDriveToTargetT = 0.0;
        // public static final double kDDriveToTargetT = 0.0;
    }


}









