package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.RobotState;
import frc.util.ShuffleboardIO;
import frc.util.Util;

// import frc.util.Constants.RobotState;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.VecBuilder;

public class SwerveDrive extends SubsystemBase {

  Vision limelight;
  private Pigeon2 gyro = new Pigeon2(RobotMap.Drivetrain.PIGEON_2);
  SwerveDriveKinematics m_kinematics;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Odometry [SD]", Pose2d.struct).publish(); // for advantage scope

  double xSpeed, ySpeed, tSpeed;
  double targetHeadingRad = Math.PI;
  double headingLockLastOffset;

  // PIDController headingController;
  PIDController speakerLockPIDController;

  public static enum Directions {
    FORWARD,
    LEFT, 
    RIGHT,
    BACK
  } 

  // initialize swervemodules
  private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.kCANCoderOffsetFrontLeft,
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
        Constants.Drivetrain.kCANCoderOffsetFrontRight,
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
        Constants.Drivetrain.kCANCoderOffsetBackLeft,
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
        Constants.Drivetrain.kCANCoderOffsetBackRight,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_RIGHT_T_IS_REVERSED,
        Constants.Drivetrain.kPTurnMotorBR,
        Constants.Drivetrain.kITurnMotorBR,
        Constants.Drivetrain.kDTurnMotorBR
  );

  public SwerveDriveOdometry odometry = new SwerveDriveOdometry( // used to track position of robot on field
    Constants.Drivetrain.kDriveKinematics,
    new Rotation2d(getHeading()),
    getModulePositions()
    );

  public SwerveDrive(Vision limelight) {
    this.limelight = limelight;

    // this.headingController = new PIDController(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);
    // this.headingController.enableContinuousInput(0, (2.0 * Math.PI));
    
    this.speakerLockPIDController = new PIDController(Constants.Drivetrain.kPIDSpeakerHeadingLock.kP, Constants.Drivetrain.kPIDSpeakerHeadingLock.kI, Constants.Drivetrain.kPIDSpeakerHeadingLock.kD);
    this.speakerLockPIDController.enableContinuousInput(0, (2.0 * Math.PI));
    this.speakerLockPIDController.setSetpoint(0.0);
    this.speakerLockPIDController.setTolerance(2.0);
    
    // Config Pathplanner
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::driveRobotRelative,
      Constants.Drivetrain.kPathFollowerConfig,
      () -> {
        // Boolean supplier for whether field is mirrored (mirrored = red)
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    },
    this
    );

    // PID tuners for individual swerve module
    ShuffleboardIO.addSlider("PID FL kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorFL);
    ShuffleboardIO.addSlider("PID FR kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorFR);
    ShuffleboardIO.addSlider("PID BL kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorBL);
    ShuffleboardIO.addSlider("PID BR kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPTurnMotorBR);
    
    ShuffleboardIO.addSlider("PID FL kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorFL);
    ShuffleboardIO.addSlider("PID FR kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorFR);
    ShuffleboardIO.addSlider("PID BL kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorBL);
    ShuffleboardIO.addSlider("PID BR kD [SD]", 0.0, 0.001, Constants.Drivetrain.kDTurnMotorBR);

    ShuffleboardIO.addSlider("PID Turn Vision kP [SD]", 0.0, 0.01, Constants.Drivetrain.kPIDSpeakerHeadingLock.getP());
    ShuffleboardIO.addSlider("PID Turn Vision kD [SD]", 0.0, 0.001, Constants.Drivetrain.kPIDSpeakerHeadingLock.getD());

    // ShuffleboardIO.addSlider("kPSwerveDriveHeading", 0, 3, Constants.Drivetrain.kPHeading);
    // ShuffleboardIO.addSlider("kISwerveDriveHeading", 0, 0.05, Constants.Drivetrain.kIHeading);
    // ShuffleboardIO.addSlider("kDSwerveDriveHeading", 0, 0.5, Constants.Drivetrain.kDHeading);

    // ShuffleboardIO.addSlider("PP kP [SD]", 0.0, 10.0, 0.0);
    // ShuffleboardIO.addSlider("PP kD [SD]", 0.0, 1.0, 0.0);
  }

  @Override
  public void periodic() {
    updateShuffleboardPIDConstants();

    debug();

    if (Constants.robotState == RobotState.TELEOP || Constants.robotState == RobotState.TELEOP_LOCK_TURN_TO_SPEAKER) {

      if (Constants.robotState == RobotState.TELEOP_LOCK_TURN_TO_SPEAKER) { // override teleop turning to lock onto a speaker 
        this.tSpeed = getLockHeadingToSpeakerTSpeed();
      }

      // set module states
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, tSpeed, new Rotation2d(getHeading()));
      SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
      setModuleStates(moduleStates);
    } 

    // update odometry with vision if we have targets + gyro is not shaky
    if (Math.abs(gyro.getRate()) < 720 && limelight.hasTargets()) {
    // if (Constants.robotState != RobotState.AUTON) {
      updateOdometryWithVision();
    }
    else { // otherwise, update with gyro
      this.odometry.update(new Rotation2d(getHeading()), getModulePositions());
    }
    // gyro.clearStickyFaults();

    Logger.recordOutput("Swerve Pose", odometry.getPoseMeters());
  }

  //------Get Methods------

  /**
   * @return Heading in radians [0, 2PI) 
   */
  public double getHeading() {
    double angle = gyro.getRotation2d().plus(Rotation2d.fromDegrees(180)).getRadians(); 
    angle *= -1.0;
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

  //------Zero Methods------

  public void zeroHeading() {
    gyro.setYaw(0);
    targetHeadingRad = Math.PI;
  }

  public void zeroDriveMotors() {
    frontLeft.zeroDriveMotor();
    frontRight.zeroDriveMotor();
    backLeft.zeroDriveMotor();
    backRight.zeroDriveMotor();
  }

  //------Reset Methods------

  public void resetModulesToAbsolute() {
    frontLeft.zeroTurnMotorABS();
    Timer.delay(0.05);
    frontRight.zeroTurnMotorABS();
    Timer.delay(0.05);
    backLeft.zeroTurnMotorABS();
    Timer.delay(0.05);
    backRight.zeroTurnMotorABS();
  }

  //------Set Neutral Mode Methods------

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

  // <-- limelight --> //

  public void updateLastOffsets() {
    headingLockLastOffset = Math.toRadians(limelight.getXOffset());
  }

  public void updateOdometryWithVision() {
    Translation2d pos = limelight.getTranslation2d();
    if (limelight.hasTargets(pos)) { // LL can see tags
      odometry.resetPosition(new Rotation2d(getHeading()), getModulePositions(), new Pose2d(pos, new Rotation2d(getHeading())));
    }
  }

  // Requires that Constants.RobotState is TELEOP_DISABLE_TURN
  public double getLockHeadingToSpeakerTSpeed() {
    // double offsetRad = Math.toRadians(Util.getSpeakerAngleOffset(odometry.getPoseMeters().getTranslation()));
    // if (Math.abs(offsetRad - headingLockLastOffset) > Constants.Drivetrain.kHeadingLockDegreeRejectionTolerance) {
    //   offsetRad = headingLockLastOffset;
    // }
    // double PIDOutput = speakerLockPIDController.calculate(offsetRad, 0.0);

    double PIDOutput = speakerLockPIDController.calculate(getHeading(), 210.0);
    if (speakerLockPIDController.atSetpoint()) {
      PIDOutput = 0.0;
    }
    double limitedOutput = Util.minmax(PIDOutput, -Constants.Drivetrain.kHeadingLockPIDMaxOutput, Constants.Drivetrain.kHeadingLockPIDMaxOutput);

    SmartDashboard.putNumber("Speaker Lock Output [SD]", limitedOutput);

    return limitedOutput;
  }

  //------PathPlanner Methods------

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setState(desiredStates[0]); //currently setting 
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);

    // SmartDashboard.putNumber("getPIDOutput", frontRight.getPIDOutput(desiredStates[1]));

    // SmartDashboard.putNumber("front left turn deg target [SD]", desiredStates[0].angle.getDegrees());
    // SmartDashboard.putNumber("front right turn deg target [SD]", desiredStates[1].angle.getDegrees());
    // SmartDashboard.putNumber("back right turn deg target [SD]", desiredStates[2].angle.getDegrees());
    // SmartDashboard.putNumber("back left turn deg target [SD]", desiredStates[3].angle.getDegrees());

    // SmartDashboard.putNumber("front left turn PID Output [SD]", frontLeft.getPIDOutput(state));
    // SmartDashboard.putNumber("front right turn PID Output [SD]", frontRight.getPIDOutput(desiredStates[1]));
    // SmartDashboard.putNumber("back right turn PID Output [SD]", backRight.getPIDOutput(state));
    // SmartDashboard.putNumber("back left turn PID Output [SD]", backLeft.getPIDOutput(state));
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

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Translation3d getTranslation3d() {
    Pose2d pose = getPose();
    return new Translation3d(pose.getX(), pose.getY(), 0.0);
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

  public void resetPose(Pose2d pose) {
    // zeroDriveMotors();
    odometry.resetPosition(new Rotation2d(getHeading()), getModulePositions(), pose);
  }

  //------Smartdashboard Methods------

  public void updateShuffleboardPIDConstants() {//
    // headingController.setP(ShuffleboardIO.getDouble("kPSwerveDriveHeading"));
    // headingController.setI(ShuffleboardIO.getDouble("kISwerveDriveHeading"));
    // headingController.setD(ShuffleboardIO.getDouble("kDSwerveDriveHeading"));

    // frontLeft.updatePIDConstants(ShuffleboardIO.getDouble("PID FL kP [SD]"), 0.0, ShuffleboardIO.getDouble("PID FL kD [SD]"));
    // frontRight.updatePIDConstants(ShuffleboardIO.getDouble("PID FR kP [SD]"), 0.0, ShuffleboardIO.getDouble("PID FR kD [SD]"));
    // backLeft.updatePIDConstants(ShuffleboardIO.getDouble("PID BL kP [SD]"), 0.0, ShuffleboardIO.getDouble("PID BL kD [SD]"));
    // backRight.updatePIDConstants(ShuffleboardIO.getDouble("PID BR kP [SD]"), 0.0, ShuffleboardIO.getDouble("PID BR kD [SD]"));

    // speakerLockPIDController.setP(ShuffleboardIO.getDouble("PID Turn Vision kP [SD]"));
    // speakerLockPIDController.setD(ShuffleboardIO.getDouble("PID Turn Vision kD [SD]"));
  }

  private void debug() {
    publisher.set(odometry.getPoseMeters());

    // SmartDashboard.putString("Robot State", Constants.robotState.toString());

    // SmartDashboard.putNumber("Target Heading (Deg) [SD]", Math.toDegrees(targetHeadingRad));
    // SmartDashboard.putNumber("Heading (Deg) [SD]", Math.toDegrees(getHeading()));
    // SmartDashboard.putNumber("Gyro Yaw", gyro.getRotation2d().getDegrees());
  
    // SmartDashboard.putNumber("Odometry X (m) [SD]", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Odometry Y (m) [SD]", odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("Odometry T (Deg) [SD]", odometry.getPoseMeters().getRotation().getDegrees());
    // SmartDashboard.putString("Odometry Pose [SD]", odometry.getPoseMeters().toString());

    // SmartDashboard.putNumber("front left cancoder degrees [SD]", frontLeft.getCANCoderDegrees());
    // SmartDashboard.putNumber("front right cancoder degrees [SD]", frontRight.getCANCoderDegrees());
    // SmartDashboard.putNumber("back right cancoder degrees [SD]", backRight.getCANCoderDegrees());
    // SmartDashboard.putNumber("back left cancoder degrees [SD]", backLeft.getCANCoderDegrees());

    // SmartDashboard.putNumber("front left turn deg [SD]", frontLeft.getTurnPosition());
    // SmartDashboard.putNumber("front right turn deg [SD]", frontRight.getTurnPosition());
    // SmartDashboard.putNumber("back right turn deg [SD]", backRight.getTurnPosition());
    // SmartDashboard.putNumber("back left turn deg [SD]", backLeft.getTurnPosition());

    // SmartDashboard.putNumber("Heading Controller PID Output [SD]", tSpeed);
    // SmartDashboard.putNumber("Speaker Offset Deg [VI]", Util.getSpeakerAngleOffset(odometry.getPoseMeters().getTranslation()));

    // <-- -->

    // SmartDashboard.putNumber("front left drive velocity rps [SD]", frontLeft.getDriveVelocity());
    // SmartDashboard.putNumber("front right drive velocity rps [SD]", frontRight.getDriveVelocity());
    // SmartDashboard.putNumber("back right drive velocity rps [SD]", backRight.getDriveVelocity());
    // SmartDashboard.putNumber("back left drive velocity rps [SD]", backLeft.getDriveVelocity());
    // SmartDashboard.putNumber("Heading Lock Turning Speed (for LL aligning) [SD]", getLockHeadingToSpeakerTSpeed());
    // SmartDashboard.putNumber("Distance to Target [SD]", getDistToTarget()); // distance is displayed in shooter pivot
    // SmartDashboard.putNumber("DEBUG - xSpeed [SD]", xSpeed);
    // SmartDashboard.putNumber("DEBUG - ySpeed [SD]", ySpeed);
    // SmartDashboard.putNumber("DEBUG - tSpeed [SD]", tSpeed);
  }

  // public boolean isAlignedWithTarget() {
  //   double offsetDeg = limelight.getXOffset();

  //   if (Math.abs(offsetDeg) < Constants.Drivetrain.kHeadingLockDegreeTolerance) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  // public double getDistToTarget() {
  //   return limelight.getDistanceToTarget(getPose().getTranslation());
  // }

  // public void calibrateGyro() {
    // gyro.calibrate();
  // }

  // public void setTargetHeadingRad(double targetHeadingRad) {
  //   this.targetHeadingRad = targetHeadingRad;
  // }

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
  //   gyro.setYaw(0);
  //   // gyro.enterCalibrationMode();
  //   // gyro.reset();
  // }

  // public double getLockHeadingToSpeakerTSpeed(double angleSpeaker) {
  //   double offsetRad = getHeading() - limelight.getSpeakerAngle();
  //   return speakerLockPIDController.calculate(offsetRad, 0.0);
  // }

  // public double getLockHeadingToSpeakerTSpeed() {
  //   Translation2d robotToTarget = limelight.getTranslationRobotToGoal();
  //   double angleOffset = Math.toDegrees(robotToTarget.getAngle().getRadians() - getHeading());
  //   speakerLockPIDController.setSetpoint(0.0);
  //   double yawSpeed = speakerLockPIDController.calculate(angleOffset);
  //   return yawSpeed;
  // }
}