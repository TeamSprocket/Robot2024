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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {

  Vision vision;

  double xSpeed, ySpeed, tSpeed;
  // double targetHeadingRad = Math.PI;
  PIDController headingController;
  SwerveDriveKinematics m_kinematics;

  public static enum Directions {
    FORWARD,
    LEFT, 
    RIGHT,
    BACK
  } 

  // private WPI_PigeonIMU gyro = new WPI_PigeonIMU(RobotMap.Drivetrain.PIGEON_2);
  private Pigeon2 gyro = new Pigeon2(RobotMap.Drivetrain.PIGEON_2);

  private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetFrontLeft,
        Constants.Drivetrain.FRONT_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_LEFT_T_IS_REVERSED,
        Constants.Drivetrain.kPTurnMotorFL,
        Constants.Drivetrain.kITurnMotorFL,
        Constants.Drivetrain.kDTurnMotorFL
  );
  private final SwerveModule frontRight = new SwerveModule(
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_D,
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_T,
        RobotMap.Drivetrain.FRONT_RIGHT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetFrontRight,
        Constants.Drivetrain.FRONT_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_RIGHT_T_IS_REVERSED,
        Constants.Drivetrain.kPTurnMotorFR,
        Constants.Drivetrain.kITurnMotorFR,
        Constants.Drivetrain.kDTurnMotorFR
  );
  private final SwerveModule backLeft = new SwerveModule(
        RobotMap.Drivetrain.BACK_LEFT_TALON_D,
        RobotMap.Drivetrain.BACK_LEFT_TALON_T,
        RobotMap.Drivetrain.BACK_LEFT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetBackLeft,
        Constants.Drivetrain.BACK_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_LEFT_T_IS_REVERSED,
        Constants.Drivetrain.kPTurnMotorBL,
        Constants.Drivetrain.kITurnMotorBL,
        Constants.Drivetrain.kDTurnMotorBL
  );
  private final SwerveModule backRight = new SwerveModule(
        RobotMap.Drivetrain.BACK_RIGHT_TALON_D,
        RobotMap.Drivetrain.BACK_RIGHT_TALON_T,
        RobotMap.Drivetrain.BACK_RIGHT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetBackRight,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_RIGHT_T_IS_REVERSED,
        Constants.Drivetrain.kPTurnMotorBR,
        Constants.Drivetrain.kITurnMotorBR,
        Constants.Drivetrain.kDTurnMotorBR
  );


  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    Constants.Drivetrain.kDriveKinematics,
    new Rotation2d(getHeading()),
    getModulePositions()
    );

  public SwerveDrive(Vision vision) {
    this.vision = vision;

    this.headingController = new PIDController(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);
    this.headingController.enableContinuousInput(0, (2.0 * Math.PI));

    this.gyro.reset();

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


    // ShuffleboardPIDTuner.addSlider("PP kP [SD]", 0.0, 10.0, 0.0);
    // ShuffleboardPIDTuner.addSlider("PP kD [SD]", 0.0, 1.0, 0.0);

    // ShuffleboardPIDTuner.addSlider("PID FL kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorFL);
    // ShuffleboardPIDTuner.addSlider("PID FR kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorFR);
    // ShuffleboardPIDTuner.addSlider("PID BL kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorBL);
    // ShuffleboardPIDTuner.addSlider("PID BR kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorBR);
    
    // ShuffleboardPIDTuner.addSlider("PID FL kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorFL);
    // ShuffleboardPIDTuner.addSlider("PID FR kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorFR);
    // ShuffleboardPIDTuner.addSlider("PID BL kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorBL);
    // ShuffleboardPIDTuner.addSlider("PID BR kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorBR);

    ShuffleboardPIDTuner.addSlider("kPSwerveDriveHeading", 0, 3, Constants.Drivetrain.kPHeading);
    // ShuffleboardPIDTuner.addSlider("kISwerveDriveHeading", 0, 0.05, Constants.Drivetrain.kIHeading);
    ShuffleboardPIDTuner.addSlider("kDSwerveDriveHeading", 0, 0.5, Constants.Drivetrain.kDHeading);

  }

  @Override
  public void periodic() {
    updateShuffleboardPIDConstants();
    gyro.clearStickyFaults();

    SmartDashboard.putNumber("DEBUG - xSpeed [SD]", xSpeed);
    SmartDashboard.putNumber("DEBUG - ySpeed [SD]", ySpeed);
    SmartDashboard.putNumber("DEBUG - tSpeed [SD]", tSpeed);
    // SmartDashboard.putNumber("Target Heading (Deg) [SD]", Math.toDegrees(targetHeadingRad));
    SmartDashboard.putNumber("Heading (Deg) [SD]", Math.toDegrees(getHeading()));

    SmartDashboard.putNumber("Gyro Yaw", gyro.getRotation2d().getDegrees());

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

    SmartDashboard.putNumber("front left drive velocity rps [SD]", frontLeft.getDriveVelocity());
    SmartDashboard.putNumber("front right drive velocity rps [SD]", frontRight.getDriveVelocity());
    SmartDashboard.putNumber("back right drive velocity rps [SD]", backRight.getDriveVelocity());
    SmartDashboard.putNumber("back left drive velocity rps [SD]", backLeft.getDriveVelocity());

    SmartDashboard.putNumber("turning speed [SD]", lockHeading());

    // SmartDashboard.putNumber("Turn PID Testing Output [SD]", frontRight.getPIDOutput(ShuffleboardPIDTuner.get("Turn Angle FR Slider [SD]"), ShuffleboardPIDTuner.get("Target Angle FR Slider [SD]")));

    

    // SmartDashboard.putNumber("front right turn deg [SD]", frontRight.getTurnMotor());



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
    double angle = gyro.getRotation2d().plus(Rotation2d.fromDegrees(180)).getRadians(); 
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


  // public void switchDirection(Directions direction) {
  //   switch (direction) {
  //     case FORWARD:
  //       targetHeadingRad = 0;
  //       break;

  //     case BACK:
  //       targetHeadingRad = Math.PI;
  //       break;

  //     case LEFT:
  //       targetHeadingRad = Math.PI / 2.0;
  //       break;

  //     case RIGHT:
  //       targetHeadingRad = 3.0 * Math.PI / 2.0;
  //       break;
  //   }
  // }
  

  // public void initGyro() {
  //   gyro.reset();
  //   gyro.reset();
  // }

  public void zeroGyro() {
    gyro.setYaw(0);
    //gyro.reset();
    // targetHeadingRad = Math.PI;
  }

  // public void calibrateGyro() {
  //   gyro.calibrate();
  // }

  // public void setTargetHeadingRad(double targetHeadingRad) {
  //   this.targetHeadingRad = targetHeadingRad;
  // }

  public void resetModulesToAbsolute() {
    frontLeft.zeroTurnMotorABS();
    Timer.delay(0.05);
    frontRight.zeroTurnMotorABS();
    Timer.delay(0.05);
    backLeft.zeroTurnMotorABS();
    Timer.delay(0.05);
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

  public void setNeutralModeDrive(NeutralModeValue neutralMode) {
    frontLeft.setNeutralModeDrive(neutralMode);
    frontRight.setNeutralModeDrive(neutralMode);
    backLeft.setNeutralModeDrive(neutralMode);
    backRight.setNeutralModeDrive(neutralMode);
  }

  public void setNeutralModeTurn(NeutralModeValue neutralMode) {
    frontLeft.setNeutralModeTurn(neutralMode);
    frontRight.setNeutralModeTurn(neutralMode);
    backLeft.setNeutralModeTurn(neutralMode);
    backRight.setNeutralModeTurn(neutralMode);
  }

  public void updateOdometryWithVision() {
    Translation2d pos = vision.getTranslation2d();
    if (vision.getIsNotVolatile()) { // LL readings not volatile
      if (vision.hasTargets(pos)) { // LL can see tags
        resetPose(new Pose2d(pos, new Rotation2d(getHeading())));
      }
    }
  }

  public double lockHeading() {
    double yaw = vision.getYaw();
    double tSpeed = vision.getTspeed(yaw);

    return tSpeed;

    // updateChassisSpeeds(0.0, 0.0, tSpeed); // TODO: test on cart
  }

  public void alignWithAprilTag() {
    double rotSpeed = vision.getYaw() * Constants.Vision.kMaxTurningSpeed * -1;
    updateChassisSpeeds(0, 0, rotSpeed);
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
    chassisSpeeds.omegaRadiansPerSecond = -chassisSpeeds.omegaRadiansPerSecond * 0.25;
    SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    moduleStates[0].speedMetersPerSecond = moduleStates[0].speedMetersPerSecond * Constants.Drivetrain.kTranslationMultPP;
    moduleStates[1].speedMetersPerSecond = moduleStates[1].speedMetersPerSecond * Constants.Drivetrain.kTranslationMultPP;
    moduleStates[2].speedMetersPerSecond = moduleStates[2].speedMetersPerSecond * Constants.Drivetrain.kTranslationMultPP;
    moduleStates[3].speedMetersPerSecond = moduleStates[3].speedMetersPerSecond * Constants.Drivetrain.kTranslationMultPP;
    // this.targetHeadingRad = getHeading();

    setModuleStates(moduleStates);
  }



  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setState(desiredStates[0]);//currently setting 
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);

    SmartDashboard.putNumber("front left turn deg target [SD]", desiredStates[0].angle.getDegrees());
    SmartDashboard.putNumber("front right turn deg target [SD]", desiredStates[1].angle.getDegrees());
    SmartDashboard.putNumber("back right turn deg target [SD]", desiredStates[2].angle.getDegrees());
    SmartDashboard.putNumber("back left turn deg target [SD]", desiredStates[3].angle.getDegrees());

    SmartDashboard.putNumber("getPIDOutput", frontRight.getPIDOutput(desiredStates[1]));

    // SmartDashboard.putNumber("front left turn PID Output [SD]", frontLeft.getPIDOutput(state));
    // SmartDashboard.putNumber("front right turn PID Output [SD]", frontRight.getPIDOutput(desiredStates[1]));
    // SmartDashboard.putNumber("back right turn PID Output [SD]", backRight.getPIDOutput(state));
    // SmartDashboard.putNumber("back left turn PID Output [SD]", backLeft.getPIDOutput(state));
  }

  public void updateChassisSpeeds(double xSpeed, double ySpeed, double tSpeed) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.tSpeed = tSpeed;

    // Removed PID Heading
    // this.targetHeadingRad += tSpeed;
    // this.targetHeadingRad %= (2.0 * Math.PI);
    // this.targetHeadingRad = (targetHeadingRad < 0) ? (targetHeadingRad + (2.0 * Math.PI)) : targetHeadingRad;
    // this.tSpeed = headingController.calculate(getHeading(), targetHeadingRad) * -1.0; // Inverted PID output because ¯\_(ツ)_/¯

  }
  public void updateShuffleboardPIDConstants() {//
    // headingController.setP(ShuffleboardPIDTuner.get("kPSwerveDriveHeading"));
    // headingController.setI(ShuffleboardPIDTuner.get("kISwerveDriveHeading"));
    // headingController.setD(ShuffleboardPIDTuner.get("kDSwerveDriveHeading"));

    // frontLeft.updatePIDConstants(ShuffleboardPIDTuner.get("kP"), 0, ShuffleboardPIDTuner.get("kPSwerveDriveHeading"));
  
    // frontLeft.updatePIDConstants(ShuffleboardPIDTuner.get("PID FL kP [SD]"), 0.0, ShuffleboardPIDTuner.get("PID FL kD [SD]"));
    // frontRight.updatePIDConstants(ShuffleboardPIDTuner.get("PID FR kP [SD]"), 0.0, ShuffleboardPIDTuner.get("PID FR kD [SD]"));
    // backLeft.updatePIDConstants(ShuffleboardPIDTuner.get("PID BL kP [SD]"), 0.0, ShuffleboardPIDTuner.get("PID BL kD [SD]"));
    // backRight.updatePIDConstants(ShuffleboardPIDTuner.get("PID BR kP [SD]"), 0.0, ShuffleboardPIDTuner.get("PID BR kD [SD]"));
  
  }

  // public void clearStickyFaults() {
    
  // }
}