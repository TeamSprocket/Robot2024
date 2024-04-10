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
import frc.util.ShuffleboardPIDTuner;
import frc.util.Util;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */
  TalonFX motor = new TalonFX(RobotMap.ShooterPivot.WRIST, "canivore");

  ShooterPivotStates state = ShooterPivotStates.NONE;
  ShooterPivotStates lastState = ShooterPivotStates.NONE;

  double motorspeed = 0.0;
  double pivotAngle = 5.0; // testing only!!

  // ProfiledpidController profiledpidController;
  // TrapezoidProfile.State goal = new TrapezoidProfile.State();
  PIDController pidController = new PIDController(Constants.ShooterPivot.kPID.kP, Constants.ShooterPivot.kPID.kI, Constants.ShooterPivot.kPID.kD);

  Supplier<Double> joystickSupplier;
  Supplier<Translation3d> botPoseSupplier;

  SendableChooser<ShooterPivotStates> selectShooterPivotState = new SendableChooser<ShooterPivotStates>();

  public enum ShooterPivotStates {
    NONE,
    STOWED,
    INTAKE,
    INDEXING,
    EJECT_NOTE,
    // SPEAKER_SUBWOOFER, SPEAKER_PODIUM, 
    SPEAKER_TEST,
    SPEAKER,
    // SPEAKER_HIGH,
    AMP,
    CROSSFIELD,
    CLIMB  
  }

  // //  Shooter table key: 
  // //    Param 1: x pos of robot (use odometry or if u wanna go fancy use swerveposeestimator)
  // //    Param 2: pivot angle (get through trial and error)
  // private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();
  // static {
  //     shooterPivotTable.put(null, null); // TODO: add values
  // }

  public ShooterPivot(Supplier<Double> joystickSupplier, Supplier<Translation3d> botPoseSupplier) {
    configMotors();

    // TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(Constants.ShooterPivot.kMaxVelocityDeg, Constants.ShooterPivot.kMaxAccelerationDeg);
    // profiledpidController = new ProfiledpidController(Constants.ShooterPivot.kPshooterPivot, Constants.ShooterPivot.kIshooterPivot, Constants.ShooterPivot.kDshooterPivot, trapezoidProfileConstraints);
    this.joystickSupplier = joystickSupplier;
    this.botPoseSupplier = botPoseSupplier;

    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30)); // test values later
    
    selectShooterPivotState.setDefaultOption("NONE", ShooterPivotStates.NONE);
    selectShooterPivotState.addOption("STOWED", ShooterPivotStates.STOWED);
    selectShooterPivotState.addOption("INTAKE", ShooterPivotStates.INTAKE);
    selectShooterPivotState.addOption("SPEAKER", ShooterPivotStates.SPEAKER);
    // selectShooterPivotState.addOption("SPEAKER_HIGH", ShooterPivotStates.SPEAKER_HIGH);
    selectShooterPivotState.addOption("SPEAKER AMP ZONE", ShooterPivotStates.SPEAKER_TEST);
    // selectShooterPivotState.addOption("SPEAKER PODIUM", ShooterPivotStates.SPEAKER_PODIUM);
    // selectShooterPivotState.addOption("SPEAKER SUBWOOFER", ShooterPivotStates.SPEAKER_SUBWOOFER);
    selectShooterPivotState.addOption("AMP", ShooterPivotStates.AMP);
    selectShooterPivotState.addOption("CLIMB", ShooterPivotStates.CLIMB);
    SmartDashboard.putData("State Selector [SP]", selectShooterPivotState);

    // ShuffleboardPIDTuner.addSlider("shooterPivot kP", 0.0, 1, Constants.ShooterPivot.kPID.kP);
    // ShuffleboardPIDTuner.addSlider("shooterPivot kD", 0.0, 0.05, 0.0);
    ShuffleboardPIDTuner.addSlider("ShooterPivot Angle [SP]", 5.0, 90.0, Constants.ShooterPivot.kTargetAngleStowed);
  }

  @Override
  public void periodic() {
    debug();

    // pidController.setP(ShuffleboardPIDTuner.get("shooterPivot kP"));
    // pidController.setD(ShuffleboardPIDTuner.get("shooterPivot kD"));

    pivotAngle = ShuffleboardPIDTuner.get("ShooterPivot Angle [SP]");

    // SmartDashboard.putString("PivotState", state.toString());
    // SmartDashboard.putNumber("shooterPivot angle [SP]", getShooterPivotAngle());
    // SmartDashboard.putNumber("shooterPivot velocity [SP]", motor.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot Target Angle [SP]", pidController.getSetpoint());
    // SmartDashboard.putNumber("Motor Speed [sp]", motorspeed);
    // setState(selectShooterPivotState.getSelected());

    switch(state) {

      case NONE:
        motor.set(0);
        break;

      case STOWED:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleStowed);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case EJECT_NOTE:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleEject);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;


      case INTAKE:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleIntake);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case INDEXING:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleIndexing);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER:
        Translation3d botPose = botPoseSupplier.get();
        if (botPose.getX() + botPose.getY() + botPose.getZ() != 0.0) {
          // double angleTarget = shooterPivotTable.get(dist); // linear interpolation
          double angleTarget = Util.getTargetShotAngleDeg(botPose, Util.getSpeakerTargetBasedOnAllianceColor());
          double angleTargetAdjusted = Constants.ShooterPivot.kHorizontalAngle - angleTarget;

          pidController.setSetpoint(angleTargetAdjusted);

          SmartDashboard.putNumber("Target Angle MECHANISM [SP]", angleTarget);
          SmartDashboard.putNumber("Target Angle ADJUSTED [SP]", angleTargetAdjusted);
        }
        // SmartDashboard.putNumber("Pivot Distance [SP]", dist);
        
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case SPEAKER_TEST:

        pidController.setSetpoint(pivotAngle); 
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);
        break;

      case CROSSFIELD:

        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleCrossfield); 
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(motorspeed);

        break;
        
      case AMP:
        pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleAmp);
        motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        
        motor.set(motorspeed);
        break;

      case CLIMB: // TODO: add limit
        double speed = joystickSupplier.get() * Constants.ShooterPivot.kManualMultiplier;
        pidController.setSetpoint(pidController.getSetpoint() + (speed));

        double finalspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

        finalspeed = Util.minmax(finalspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
        motor.set(finalspeed);
        break;
    }

    // clearStickyFaults();
    lastState = state;
    SmartDashboard.putNumber("ShooterPivot Angle [SP]", getShooterPivotAngle());

    // <-- delete later -->

    // case SPEAKER_SUBWOOFER:
      //   pidController.setSetpoint(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer);
      //   motorspeed = pidController.calculate(getShooterPivotAngle()) + Constants.ShooterPivot.kPID.kFF;

      //   motorspeed = Util.minmax(motorspeed, -1 * Constants.ShooterPivot.kMaxShooterPivotOutput, Constants.ShooterPivot.kMaxShooterPivotOutput);
      //   motor.set(motorspeed);
      //   break;
  }

  /**
   * TODO: Test CW or CCW signage 
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
  }

  public boolean atGoal() {
    double goal = pidController.getSetpoint();
    return Util.inRange(getShooterPivotAngle(), (goal - Constants.ShooterPivot.kAtGoalTolerance), (goal + Constants.ShooterPivot.kAtGoalTolerance));
  }
  
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  private void configMotors() {
    // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    // currentLimitsConfigs.withSupplyCurrentLimit(Constants.ShooterPivot.kSupplyCurrentLimit);
    // currentLimitsConfigs.withSupplyCurrentLimitEnable(true);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // motorConfig.withCurrentLimits(currentLimitsConfigs);

    motor.getConfigurator().apply(motorConfig);
  }
}
