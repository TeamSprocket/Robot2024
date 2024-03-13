package frc.robot.subsystems;

import javax.sound.sampled.TargetDataLine;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.util.ISO8601Utils;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class bigRichardSwerveDrive extends SubsystemBase {
    Timer timer;
    boolean correct;
    double targetHeading;
    PIDController pidController = new PIDController(kPHeading, kIHeading, kDHeading);
    double xSpeed;
    double ySpeed; 
    double zSpeed;

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

        headingController.enableContinuousInput(0,360);

    }

    public void zeroTalons() {
        frontLeft.zeroTalon();
        frontRight.zeroTalon();
        backLeft.zeroTalon();
        backRight.zeroTalon();
        frontLeft.zeroDrive();
        frontRight.zeroDrive();
        backLeft.zeroDrive();
        backRight.zeroDrive();
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

    public void setTurnDefaultMode(NeutralMode mode) {
        frontLeft.setTurnDefaultMode(mode);
        frontRight.setTurnDefaultMode(mode);
        backLeft.setTurnDefaultMode(mode);
        backRight.setTurnDefaultMode(mode);
    }

    public void setDriveDefaultMode(NeutralMode mode) {
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

    public void updateHeading(Direction direction) {
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
        double diff = Math.abs(timer.get() - last);
        last = timer.get();

        this.targetHeading += tSpeed * Constants.Drivetrain.kMaxTurnSpeed;
        this.targetHeading = (targetHeading % 360.0);
        if (targetHeading < 0){
            this.targetHeading = targetHeading + 360.0;
        }
        else {
            this.targetHeading = targetHeading;
        }

        headingController.setSetpoint(targetHeading);
        double tSpeedPID = headingController.calculate(getHeading());

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.tSpeed = tSpeedPID;
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
                    xSpeed, ySpeed, tSpeed, new Rotation2d(headingRad));

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

