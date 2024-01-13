
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.RobotState;
import frc.util.ShuffleboardPIDTuner;

public class SwerveDrive extends SubsystemBase {

  double xSpeed, ySpeed, tSpeed;
  double targetHeadingRad = Math.PI;
  PIDController headingController;


  public static enum Directions {
    FORWARD,
    LEFT, 
    RIGHT,
    BACK
  } 

  
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  
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

  public SwerveDrive() {
    this.headingController = new PIDController(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);
    this.headingController.enableContinuousInput(0, (2.0 * Math.PI));

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
        if (!alliance.equals(DriverStation.Alliance.Invalid)) {
            return alliance.equals(DriverStation.Alliance.Red);
        }
        return false;
    },
    this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DEBUG - xSpeed", xSpeed);
    SmartDashboard.putNumber("DEBUG - ySpeed", ySpeed);
    SmartDashboard.putNumber("DEBUG - tSpeed", tSpeed);
    SmartDashboard.putNumber("Target Heading (Deg)", Math.toDegrees(targetHeadingRad));
    SmartDashboard.putNumber("Heading (Deg)", Math.toDegrees(getHeading()));
    SmartDashboard.putNumber("Odometry X (m)", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y (m)", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry T (Deg)", odometry.getPoseMeters().getRotation().getDegrees());

    
    if (Constants.robotState == RobotState.TELEOP) {
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, tSpeed, new Rotation2d(getHeading()));
      SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
      setModuleStates(moduleStates);
    }


    // Update Odometer
    
    this.odometry.update(new Rotation2d(getHeading()), getModulePositions());


    
    
  }


  /**
   * @return Heading in radians [0, 2PI) 
   */
  public double getHeading() { // ? why 0
    double angle = gyro.getAngle() + 180.0;
    
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
  

  public void initGyro() {
    gyro.reset();
    gyro.calibrate();
    gyro.reset();
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

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

  // Stuff for Pathplanner
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(new Rotation2d(getHeading()), getModulePositions(), getPose());
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
    SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    frontLeft.setState(moduleStates[0]);
    frontLeft.setState(moduleStates[0]);
    frontLeft.setState(moduleStates[0]);
    frontLeft.setState(moduleStates[0]);
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



  




}











