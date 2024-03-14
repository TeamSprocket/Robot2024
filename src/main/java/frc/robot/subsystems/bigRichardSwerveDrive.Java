package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;

public class bigRichardSwerveDrive extends SubsystemBase {
    Timer timer;
    boolean correct;
    double targetHeading;
    PIDController pidController = new PIDController(Constants.bigRichardDriveTrain.kPHeading, Constants.bigRichardDriveTrain.kIHeading, Constants.bigRichardDriveTrain.kDHeading);
    double xSpeed;
    double ySpeed; 
    double zSpeed;
    double num = 0.0;

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    public void toggleCorrect() {
        this.correct = !this.correct;
    }

    public void zeroHeading (){
        gyro.reset();
        this.targetHeading = 0;
    }

    public void updateTargetHeading() {
        this.targetHeading = getHeading();
    }

    public bigRichardSwerveDrive() {
        this.timer = new Timer();
        timer.reset();

        pidController.enableContinuousInput(0,360);

    }

    public void zeroTalons() {
        frontLeft.zeroTurnMotorABS();
        frontRight.zeroTurnMotorABS();
        backLeft.zeroTurnMotorABS();
        backRight.zeroTurnMotorABS();
        frontLeft.zeroDriveMotor();
        frontRight.zeroDriveMotor();
        backLeft.zeroDriveMotor();
        backRight.zeroDriveMotor();
    }

    public void clearStickyFaults() {
        frontLeft.clearStickyFaults();
        frontRight.clearStickyFaults();
        backLeft.clearStickyFaults();
        backRight.clearStickyFaults();
    }

    public void setCurrentLimitTurn(double currentLimit) {
        frontLeft.setCurrentLimitTurn(currentLimit);
        frontRight.setCurrentLimitTurn(currentLimit);
        backLeft.setCurrentLimitTurn(currentLimit);
        backRight.setCurrentLimitTurn(currentLimit);
    }

    public void setCurrentLimitDrive(double currentLimit) {
        frontLeft.setCurrentLimitDrive(currentLimit);
        frontRight.setCurrentLimitDrive(currentLimit);
        backLeft.setCurrentLimitDrive(currentLimit);
        backRight.setCurrentLimitDrive(currentLimit);
    }



    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = {new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition()))
        };
        return swerveModulePositions;
    }

    public void setTurnDefaultMode(NeutralModeValue mode) {
        frontLeft.setNeutralModeTurn(mode);
        frontRight.setNeutralModeTurn(mode);
        backLeft.setNeutralModeTurn(mode);
        backRight.setNeutralModeTurn(mode);
    }

    public void setDriveDefaultMode(NeutralModeValue mode) {
        frontLeft.setDriveDefaultMode(mode);
        frontRight.setDriveDefaultMode(mode);
        backLeft.setDriveDefaultMode(mode);
        backRight.setDriveDefaultMode(mode);
    }

    public void calibrateGyro() {
        gyro.calibrate();
    }

    public double getPitchDegFiltered() {
        double deg = gyro.getYComplementaryAngle();
        deg -= 360;
        deg %= 360;
        deg = Math.abs(deg);

        if (deg > 180) { 
            deg = 0;
        }
        return deg;
    }

    public double getPitchDeg() {
        double deg = gyro.getYComplementaryAngle();
        deg %= 360;
        if (deg >= 180) {
            deg -= 360;
        }

        double defaultOffset = -1.8;
        deg -= defaultOffset;

        return -deg;
    
    }

    public double getHeading() {
        double angle = gyro.getAngle() % 360.0;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public double getHeadingRad() {
        return Math.toRadians(getHeading());
    }

    public void updateHeading(Directions direction) {
        switch (direction) {
            case FRONT: 
                targetHeading = 180;
                break;
            case LEFT: 
                targetHeading = 270;
                break;
            case RIGHT: 
                targetHeading = 90;
                break;
            case BACK: 
                targetHeading = 0;
                break;
        }
    }

    public void setModuleSpeeds(double xSpeed, double ySpeed, double tSpeed) {
        double diff = Math.abs(timer.get() - num);
        num = timer.get();

        SmartDashboard.putNumber("FrontLeftAngleTarget", moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("FrontRightAngleTarget", desiredStates[1].angle.getDegrees());
        SmartDashboard.putNumber("BackLeftAngleTarget", desiredStates[2].angle.getDegrees());
        SmartDashboard.putNumber("BackRightAngleTarget", desiredStates[3].angle.getDegrees());
        SmartDashboard.putNumber("FrontLeftAngleTalonEncoder", Math.toDegrees(frontLeft.getTurnPosition()));
        SmartDashboard.putNumber("FrontRightAngleTalonEncoder", Math.toDegrees(frontRight.getTurnPosition()));
        SmartDashboard.putNumber("BackLeftAngleTalonEncoder", Math.toDegrees(backLeft.getTurnPosition()));
        SmartDashboard.putNumber("BackRightAngleTalonEncoder", Math.toDegrees(backRight.getTurnPosition()));

        SmartDashboard.putNumber("Gyro", getHeading());

        this.targetHeading += tSpeed * Constants.Drivetrain.kMaxTurnSpeed;
        this.targetHeading = (targetHeading % 360.0);
        if (targetHeading < 0){
            this.targetHeading = targetHeading + 360.0;
        }
        else {
            this.targetHeading = targetHeading;
        }

        pidController.setSetpoint(targetHeading);
        double tSpeedPID = pidController.calculate(getHeading());

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.xSpeed = tSpeedPID;
    }

    public enum Directions{
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.bigRichardDriveTrain.FRONT_LEFT_TALON_D,
        RobotMap.bigRichardDriveTrain.FRONT_LEFT_TALON_T
        Constants.bigRichardDriveTrain.FRONT_LEFT_D_IS_REVERSED,
        Constants.bigRichardDriveTrain.FRONT_LEFT_T_IS_REVERSED,
        RobotMap.bigRichardDriveTrain.FRONT_RIGHT_ABS_ENCODER_ID,
        Constants.bigRichardDriveTrain.FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD,
        true
        );

    private final SwerveModule frontRight = new SwerveModule(
        RobotMap.bigRichardDriveTrain.FRONT_RIGHT_TALON_D,
        RobotMap.bigRichardDriveTrain.FRONT_RIGHT_TALON_T
        Constants.bigRichardDriveTrain.FRONT_RIGHT_D_IS_REVERSED,
        Constants.bigRichardDriveTrain.FRONT_RIGHT_T_IS_REVERSED,
        RobotMap.bigRichardDriveTrain.BACK_RIGHT_ABS_ENCODER_ID,
        Constants.bigRichardDriveTrain.BACK_RIGHT_ABS_ENCODER_OFFSET_RAD,
        false
        );

    private final SwerveModule backLeft = new SwerveModule(
        RobotMap.bigRichardDriveTrain.BACK_LEFT_TALON_D,
        RobotMap.bigRichardDriveTrain.BACK_LEFT_TALON_T
        Constants.bigRichardDriveTrain.BACK_LEFT_D_IS_REVERSED,
        Constants.bigRichardDriveTrain.BACK_LEFT_T_IS_REVERSED,
        RobotMap.bigRichardDriveTrain.FRONT_LEFT_ABS_ENCODER_ID,
        Constants.bigRichardDriveTrain.FRONT_LEFT_ABS_ENCODER_OFFSET_RAD,
        false
        );

    private final SwerveModule backRight = new SwerveModule(
        RobotMap.bigRichardDriveTrain.BACK_RIGHT_TALON_D,
        RobotMap.bigRichardDriveTrain.BACK_RIGHT_TALON_T
        Constants.bigRichardDriveTrain.BACK_RIGHT_D_IS_REVERSED,
        Constants.bigRichardDriveTrain.BACK_RIGHT_T_IS_REVERSED,
        RobotMap.bigRichardDriveTrain.BACK_LEFT_ABS_ENCODER_ID,
        Constants.bigRichardDriveTrain.BACK_LEFT_ABS_ENCODER_OFFSET_RAD,
        true
        );

        @Override
    public void periodic() {

        SmartDashboard.putNumber("Target Heading", targetHeading);

        clearStickyFaults();

        if (Constants.isEnabled) {
            double headingRad = Math.toRadians(-getHeading());
            if (Constants.Auton.kFacingDriversOnStart) {
                headingRad += Math.PI;
            }

            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, xSpeed, new Rotation2d(headingRad));

            SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);

            frontLeft.setDesiredState(moduleStates[0], isPrecise);
            frontRight.setDesiredState(moduleStates[1], isPrecise);
            backLeft.setDesiredState(moduleStates[2], isPrecise);
            backRight.setDesiredState(moduleStates[3], isPrecise);
        }
    }

    public void postShuffleboardPIDTuner() {
        headingController.setP(ShuffleboardPIDTuner.get("kPSwerveDriveHeading"));
        headingController.setI(ShuffleboardPIDTuner.get("kISwerveDriveHeading"));
        headingController.setD(ShuffleboardPIDTuner.get("kDSwerveDriveHeading"));
        Constants.Drivetrain.kMaxTurnSpeed = ShuffleboardPIDTuner.get("kMaxSpeedAngular");
        Constants.Drivetrain.kMaxTurnAccel = ShuffleboardPIDTuner.get("kMaxAccelAngular");
    }

    public double getFrontLeftTicks() {
        return frontLeft.getDrivePosition();
    }

    public SwerveModuleState[] getDesiredStates() {
        return states;
    }

}

