
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.hardware.core.CorePigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

public class SwerveDrive extends SubsystemBase {

  Limelight limelight;

  double xSpeed, ySpeed, tSpeed;
  double targetHeadingRad = Math.PI;
  PIDController headingController;
  SwerveDriveKinematics m_kinematics;

  public static enum Directions {
    FORWARD,
    LEFT, 
    RIGHT,
    BACK
  } 

  
  private PigeonIMU gyro = new PigeonIMU(RobotMap.Drivetrain.PIGEON_2);
  
  private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetFrontLeft,
        Constants.Drivetrain.FRONT_LEFT_D_IS_REVERSED
  );
  private final SwerveModule frontRight = new SwerveModule(
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_D,
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_T,
        RobotMap.Drivetrain.FRONT_RIGHT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetFrontRight,
        Constants.Drivetrain.FRONT_RIGHT_D_IS_REVERSED
  );
  private final SwerveModule backLeft = new SwerveModule(
        RobotMap.Drivetrain.BACK_LEFT_TALON_D,
        RobotMap.Drivetrain.BACK_LEFT_TALON_T,
        RobotMap.Drivetrain.BACK_LEFT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetBackLeft,
        Constants.Drivetrain.BACK_LEFT_D_IS_REVERSED
  );
  private final SwerveModule backRight = new SwerveModule(
        RobotMap.Drivetrain.BACK_RIGHT_TALON_D,
        RobotMap.Drivetrain.BACK_RIGHT_TALON_T,
        RobotMap.Drivetrain.BACK_RIGHT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetBackRight,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED
  );


  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    Constants.Drivetrain.kDriveKinematics,
    new Rotation2d(getHeading()),
    getModulePositions()
    );

  public SwerveDrive(Limelight limelight) {
    this.limelight = limelight;

    this.headingController = new PIDController(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);
    this.headingController.enableContinuousInput(0, (2.0 * Math.PI));

    ShuffleboardPIDTuner.addSlider("Swerve PID kP [SD]", -0.1, 0.1, 0);
    ShuffleboardPIDTuner.addSlider("Swerve PID kD [SD]", -0.01, 0.01, 0);

    // ShuffleboardPIDTuner.addSlider("kPSwerveDriveHeading", 0, 0.05, Constants.Drivetrain.kPTranslationPP);
    // ShuffleboardPIDTuner.addSlider("kISwerveDriveHeading", 0, 0.05, Constants.Drivetrain.kITranslationPP);
    // ShuffleboardPIDTuner.addSlider("kDSwerveDriveHeading", 0, 0.05, Constants.Drivetrain.kDTranslationPP);

    // Config Pathplanner
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::driveRobotRelative,
      Constants.Drivetrain.kPathFollowerConfig,
      () -> {
        // Boolean supplier for whether field is mirrored (mirrored = on red)
        var alliance = DriverStation.getAlliance();
        if (alliance.equals(DriverStation.Alliance.Blue) || alliance.equals(DriverStation.Alliance.Red)) {
            return alliance.equals(DriverStation.Alliance.Red);
        }
        return false;
    },
    this
    );



  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DEBUG - xSpeed [SD]", xSpeed);
    SmartDashboard.putNumber("DEBUG - ySpeed [SD]", ySpeed);
    SmartDashboard.putNumber("DEBUG - tSpeed [SD]", tSpeed);
    SmartDashboard.putNumber("Target Heading (Deg) [SD]", Math.toDegrees(targetHeadingRad));
    SmartDashboard.putNumber("Heading (Deg) [SD]", Math.toDegrees(getHeading()));
    SmartDashboard.putNumber("Odometry X (m) [SD]", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y (m) [SD]", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry T (Deg) [SD]", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putString("Odometry Pose [SD]", odometry.getPoseMeters().toString());

    SmartDashboard.putNumber("front left cancoder [SD]", frontLeft.getCANCoderDegrees());
    SmartDashboard.putNumber("front right cancoder [SD]", frontRight.getCANCoderDegrees());
    SmartDashboard.putNumber("back right cancoder [SD]", backRight.getCANCoderDegrees());
    SmartDashboard.putNumber("back left cancoder [SD]", backLeft.getCANCoderDegrees());

    SmartDashboard.putNumber("front left turn deg [SD]", frontLeft.getTurnPosition());
    SmartDashboard.putNumber("front right turn deg [SD]", frontRight.getTurnPosition());
    SmartDashboard.putNumber("back right turn deg [SD]", backRight.getTurnPosition());
    SmartDashboard.putNumber("back left turn deg [SD]", backLeft.getTurnPosition());

    Constants.Drivetrain.kPTurnMotor = ShuffleboardPIDTuner.get("Swerve PID kP [SD]");
    Constants.Drivetrain.kDTurnMotor = ShuffleboardPIDTuner.get("Swerve PID kD [SD]");

    // SmartDashboard.putNumber("front right turn deg [SD]", frontRight.getTurnMotor());


    // updateShuffleboardPIDConstants();

    if (Constants.robotState == RobotState.TELEOP) {
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, tSpeed, new Rotation2d(getHeading()));
      SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
      setModuleStates(moduleStates);
    }

    // Update Odometer
    this.odometry.update(new Rotation2d(getHeading()), getModulePositions());
    // updateOdometryWithVision();
    
  }



  /**
   * @return Heading in radians [0, 2PI) 
   */
  public double getHeading() { // ? why 0
    double angle = gyro.getYaw() + 180.0; 
    
    angle %= 360.0;
    if (angle < 0) {
        angle += 360;
    }

    angle = Math.toRadians(angle);

    return angle;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = {
      new SwerveModulePosition(frontLeft.getDrivePosMeters(), new Rotation2d(frontLeft.getTurnRad())),
      new SwerveModulePosition(frontRight.getDrivePosMeters(), new Rotation2d(frontRight.getTurnRad())),
      new SwerveModulePosition(backLeft.getDrivePosMeters(), new Rotation2d(backLeft.getTurnRad())),
      new SwerveModulePosition(backRight.getDrivePosMeters(), new Rotation2d(backRight.getTurnRad())),
    };
    return modulePositions;
  }


  public void switchDirection(Directions direction) {
    switch (direction) {
      case FORWARD:
        targetHeadingRad = 0;
        break;

      case BACK:
        targetHeadingRad = Math.PI;
        break;

      case LEFT:
        targetHeadingRad = Math.PI / 2.0;
        break;

      case RIGHT:
        targetHeadingRad = 3.0 * Math.PI / 2.0;
        break;
    }
  }
  

  // public void initGyro() {
  //   gyro.setYaw(0);
  //   // gyro.enterCalibrationMode();
  //   // gyro.reset();
  // }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  // public void calibrateGyro() {
    // gyro.calibrate();
  // }

  public void setTargetHeadingRad(double targetHeadingRad) {
    this.targetHeadingRad = targetHeadingRad;
  }

  public void resetModulesToAbsolute() {
    frontLeft.zeroTurnMotorABS();
    Timer.delay(0.1);
    frontRight.zeroTurnMotorABS();
    Timer.delay(0.1);
    backLeft.zeroTurnMotorABS();
    Timer.delay(0.1);
    backRight.zeroTurnMotorABS();
  }

  public void zeroDriveMotors() {
    frontLeft.zeroDriveMotor();
    frontRight.zeroDriveMotor();
    backLeft.zeroDriveMotor();
    backRight.zeroDriveMotor();
  }

  public void setNeutralMode(NeutralModeValue neutralMode) {
    frontLeft.setNeutralMode(neutralMode);
    frontRight.setNeutralMode(neutralMode);
    backLeft.setNeutralMode(neutralMode);
    backRight.setNeutralMode(neutralMode);
  }

  public void updateOdometryWithVision() {
    Translation2d pos = limelight.getTranslation2d();
    if (limelight.getIsNotVolatile()) { // LL readings not volatile
      if (Math.abs(pos.getX()) > 0.1 && Math.abs(pos.getX()) > 0.1) { // LL can see tags
        resetPose(new Pose2d(pos, new Rotation2d(getHeading())));
      }
    }
  }

  // Stuff for Pathplanner
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    // zeroDriveMotors();
    odometry.resetPosition(new Rotation2d(getHeading()), getModulePositions(), pose);
    // odometry.
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] moduleStates = {
      frontLeft.getModuleState(),
      frontRight.getModuleState(),
      backLeft.getModuleState(),
      backRight.getModuleState()
    };
    return Constants.Drivetrain.kDriveKinematics.toChassisSpeeds(moduleStates);
  }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds chassisSpeeds = robotRelativeSpeeds; 
    chassisSpeeds.omegaRadiansPerSecond = -chassisSpeeds.omegaRadiansPerSecond;
    SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    this.targetHeadingRad = getHeading();

    setModuleStates(moduleStates);
  }



  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setState(desiredStates[0]);//currently setting 
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  public void updateChassisSpeeds(double xSpeed, double ySpeed, double tSpeed) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;

    this.targetHeadingRad += tSpeed;
    this.targetHeadingRad %= (2.0 * Math.PI);
    this.targetHeadingRad = (targetHeadingRad < 0) ? (targetHeadingRad + (2.0 * Math.PI)) : targetHeadingRad;
    this.tSpeed = headingController.calculate(getHeading(), targetHeadingRad) * -1.0; // Inverted PID output because ¯\_(ツ)_/¯

  }
  public void updateShuffleboardPIDConstants() {//
    headingController.setP(ShuffleboardPIDTuner.get("kPSwerveDriveHeading"));
    headingController.setI(ShuffleboardPIDTuner.get("kISwerveDriveHeading"));
    headingController.setD(ShuffleboardPIDTuner.get("kDSwerveDriveHeading"));
  }

  // public void clearStickyFaults() {
    
  // }
}