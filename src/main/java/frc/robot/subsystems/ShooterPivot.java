// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.pidController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
// import frc.robot.subsystems.Intake.IntakeStates;
import frc.util.Conversions;
import frc.util.ShuffleboardIO;
import frc.util.Util;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */
  TalonFX motor = new TalonFX(RobotMap.ShooterPivot.WRIST, "canivore");

  double motorspeed = 0.0;

  double targetPivotAngle = 0.0;

  double distTestXSpeaker = 0.0;
  double distTestYSpeaker = 0.0;
  double visionAngle = 0.0;

  // ProfiledpidController profiledpidController;
  // TrapezoidProfile.State goal = new TrapezoidProfile.State();
  PIDController pidController = new PIDController(Constants.ShooterPivot.kPID.kP, Constants.ShooterPivot.kPID.kI, Constants.ShooterPivot.kPID.kD);

  Supplier<Double> joystickSupplier;
  Supplier<Pose2d> robotPose;


  SendableChooser<ShooterPivotStates> selectShooterPivotState = new SendableChooser<ShooterPivotStates>();

  /**
   * Individual shooterPivot states; for each state, different things are activated to satisfy each state.
   */
  public enum ShooterPivotStates {
    NONE,
    STOWED,
    INTAKE,
    INDEXING,
    EJECT_NOTE,
    SPEAKER_PODIUM,
    SPEAKER_AMP_ZONE,
    SPEAKER_SUBWOOFER,
    SPEAKER,
    // SPEAKER_HIGH,
    AMP,
    CROSSFIELD,
    CLIMB  
  }

  ShooterPivotStates state = ShooterPivotStates.NONE;
  ShooterPivotStates lastState = ShooterPivotStates.NONE;

  // //  Shooter table key: 
  // //    Param 1: x pos of robot (use odometry or if u wanna go fancy use swerveposeestimator)
  // //    Param 2: pivot angle (get through trial and error)
  // private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();
  // static {
  //     shooterPivotTable.put(null, null); // TODO: add values
  // }

  /**
   * Constructor for shooterPivot
   * @param joystickSupplier
   * @param botPoseSupplier
   */
  public ShooterPivot(Supplier<Double> joystickSupplier, Supplier<Pose2d> robotPose) {
    configMotors();

    // TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.ShooterPivot.kMaxVelocityDeg, Constants.ShooterPivot.kMaxAccelerationDeg);
    // profiledpidController = new ProfiledpidController(Constants.ShooterPivot.kPshooterPivot, Constants.ShooterPivot.kIshooterPivot, Constants.ShooterPivot.kDshooterPivot, trapezoidProfileConstraints);
    this.joystickSupplier = joystickSupplier;
    this.robotPose = robotPose;

    //configures the motors
    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30)); // test values later
    
    pidController.setTolerance(Constants.ShooterPivot.kAtGoalTolerance);

    //Adds options for the selection thing
    selectShooterPivotState.setDefaultOption("NONE", ShooterPivotStates.NONE);
    selectShooterPivotState.addOption("STOWED", ShooterPivotStates.STOWED);
    selectShooterPivotState.addOption("INTAKE", ShooterPivotStates.INTAKE);
    selectShooterPivotState.addOption("SPEAKER", ShooterPivotStates.SPEAKER);
    // selectShooterPivotState.addOption("SPEAKER_HIGH", ShooterPivotStates.SPEAKER_HIGH);
    selectShooterPivotState.addOption("SPEAKER AMP ZONE", ShooterPivotStates.SPEAKER_AMP_ZONE);
    selectShooterPivotState.addOption("SPEAKER PODIUM", ShooterPivotStates.SPEAKER_PODIUM);
    // selectShooterPivotState.addOption("SPEAKER SUBWOOFER", ShooterPivotStates.SPEAKER_SUBWOOFER);
    selectShooterPivotState.addOption("AMP", ShooterPivotStates.AMP);
    selectShooterPivotState.addOption("CLIMB", ShooterPivotStates.CLIMB);
    SmartDashboard.putData("State Selector [SP]", selectShooterPivotState);

    ShuffleboardIO.addSlider("shooterPivot kP", 0.0, 0.01, Constants.ShooterPivot.kPID.kP);
    ShuffleboardIO.addSlider("shooterPivot kD", 0.0, 0.001, 0.0);
    ShuffleboardIO.addSlider("kHorizontalAngle ShooterPivot [SP]", 5.0, 90.0, Constants.ShooterPivot.kTargetAngleStowed);

    // ShuffleboardIO.addSlider("kStartingOffsetAngleDeg [SP]", -5.0, 5, 0.0);
    ShuffleboardIO.addSlider("kTargetAngleAmp [SP]", 60.0, 120, Constants.ShooterPivot.kTargetAngleAmp);
  }

  @Override
  public void periodic() {
    debug();

    // Constants.ShooterPivot.kTargetAngleAmp = ShuffleboardIO.getDouble("kTargetAngleAmp [SP]");

    // pidController.setP(ShuffleboardIO.getDouble("shooterPivot kP"));
    // pidController.setD(ShuffleboardIO.getDouble("shooterPivot kD"));

    // Constants.ShooterPivot.kStarting` = ShuffleboardIO.getDouble("kStartingOffsetAngleDeg [SP]");
    // Constants.ShooterPivot.kHorizontalAngle = ShuffleboardIO.getDouble("kHorizontalAngle ShooterPivot [SP]");

    // SmartDashboard.putString("PivotState", state.toString());
    // SmartDashboard.putNumber("shooterPivot angle [SP]", getShooterPivotAngle());
    // SmartDashboard.putNumber("shooterPivot velocity [SP]", motor.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot Target Angle [SP]", pidController.getSetpoint());
    // SmartDashboard.putNumber("Motor Speed [sp]", motorspeed);
    // setState(selectShooterPivotState.getSelected());

    //switches the state
    switch(state) {

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleStowed);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case EJECT_NOTE:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleEject);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;


      case INTAKE:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleIntake);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case INDEXING:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleIndexing);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case SPEAKER:
        motorspeed = getPivotSpeed(targetPivotAngle);

        motor.set(motorspeed);
        
        break;

      case SPEAKER_SUBWOOFER:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer);

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
      break;

      case SPEAKER_AMP_ZONE:

        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleSpeakerFromAmpZone); 

        motor.set(motorspeed);
        break;

      case SPEAKER_PODIUM:

        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAnglePodium);
        motor.set(motorspeed);

        break;
 
      case CROSSFIELD:

        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleCrossfield); 

        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);

        break;
        
      case AMP:
        motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleAmp);

        
        motor.set(motorspeed);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case CLIMB: // TODO: add limit
        double speed = joystickSupplier.get() * Constants.ShooterPivot.kManualMultiplier;
        motorspeed = getPivotSpeed(pidController.getSetpoint() + (speed));

        motor.set(motorspeed);
        break;
    }

    // clearStickyFaults();
    lastState = state;
    SmartDashboard.putNumber("ShooterPivot Angle [SP]", getShooterPivotAngle());
    SmartDashboard.putBoolean("Shooter Pivot atGoal [SP]", atGoal());
    SmartDashboard.putNumber("ShooterPivot TargetVision [SP]", shooterPivotAngleVision());

    // case SPEAKER_SUBWOOFER:
      //   pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer);
      //   motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

      //   motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
      //   motor.set(motorspeed);
      //   break;
  }

  /**
   * @return Angle of the shooterPivot in degrees, CW+, CCW-
   */

  public double getShooterPivotAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.ShooterPivot.kShooterPivotGearRatio);

    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }

  public double shooterPivotAngleVision() {
    Pose2d pose = robotPose.get();
    double speakerY = Constants.FieldConstants.kSpeakerTargetHeightMeters - 0.64;
    double distanceToSpeaker = Math.sqrt(Math.pow(16 - pose.getX(), 2) + Math.pow(5.5 - pose.getY(), 2));
    distanceToSpeaker = distanceToSpeaker - 0.155;

    distTestXSpeaker = distanceToSpeaker;
    distTestYSpeaker = speakerY;

    double angle = Math.atan(speakerY / distanceToSpeaker);
    angle = Math.toDegrees(angle);
    visionAngle = angle;

    angle = -1 * angle;
    angle = 47.5 + angle;
    if (angle < 0) angle = 0;
    return angle;
}

  /**
   * Gets the pivot speed
   * @param targetAngle Uses the targetAngle to get a set pivot speed.
   * @return
   */
  public double getPivotSpeed(double targetAngle) {
    double pivotSpeed;
    pidController.setSetpoint(targetAngle);
    double currentAngle = getShooterPivotAngle();
    double PIDoutput = pidController.calculate(currentAngle);
    double signedKFF = Constants.ShooterPivot.kPID.kFF * Util.getSign(PIDoutput);

    if(Math.abs(targetAngle - currentAngle) > Constants.ShooterPivot.kFFtoPIDTransitionTolerance) {
      pivotSpeed = Constants.ShooterPivot.kFFPivot * Util.getSign(PIDoutput);
    } else {
      pivotSpeed = PIDoutput + signedKFF;
      if (pidController.atSetpoint()) {
        pivotSpeed = 0;
      }
    }

    pivotSpeed = Util.minmax(pivotSpeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
    return pivotSpeed;
  }

  public void setState(ShooterPivotStates state) {
    this.state = state;
  }

  public ShooterPivotStates getState() {
      return state;
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    motor.setNeutralMode(neutralModeValue);
  }

  // public double getShooterPivotTargetDeg() {
  //   double dist = Conversions.poseToDistance(botPoseSupplier.get(), Constants.ShootingSetpoints.targetPoint);
  //   double targetDeg = Constants.ShootingSetpoints.getValues(dist)[0];
  //   return targetDeg;
  // }

  public void debug() {
    SmartDashboard.putNumber("Angle in Degrees", getShooterPivotAngle());
    // SmartDashboard.putNumber("Shot Target Angle [SP]", Util.getTargetShotAngleDeg(botPoseSupplier.get(), Util.getSpeakerTargetBasedOnAllianceColor()));
    SmartDashboard.putString("State [SP]", state.toString());
  }

  /**
   * 
   * @return If the shooterPivot is at the setpoint.
   */
  public boolean atGoal() {
    // double goal = pidController.getSetpoint();
    // return Util.inRange(getShooterPivotAngle(), (goal - Constants.ShooterPivot.kAtGoalTolerance), (goal + Constants.ShooterPivot.kAtGoalTolerance));
    return pidController.atSetpoint();
  }

  public void zeroPosition() {
    motor.setPosition(0);
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  public void setTargetPivotAngle(double target) {
    targetPivotAngle = target;
  }

  //Configurate the motors.
  private void configMotors() {
    // current limiting configs

    // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    // currentLimitsConfigs.withSupplyCurrentLimit(Constants.ShooterPivot.kSupplyCurrentLimit);
    // currentLimitsConfigs.withSupplyCurrentLimitEnable(true);

    // motorConfig.withCurrentLimits(currentLimitsConfigs);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();


    motor.getConfigurator().apply(motorConfig);
  }
}
