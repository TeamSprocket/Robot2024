// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import frc.util.ShuffleboardIO;
import frc.util.Util;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */
  TalonFX motor = new TalonFX(RobotMap.ShooterPivot.WRIST, "canivore");

  double motorspeed = 0.0;
  
  // ProfiledpidController profiledpidController;
  // TrapezoidProfile.State goal = new TrapezoidProfile.State();
  PIDController pidController = new PIDController(Constants.ShooterPivot.kPID.kP, Constants.ShooterPivot.kPID.kI, Constants.ShooterPivot.kPID.kD);

  Supplier<Double> joystickSupplier;
  Supplier<Translation3d> botPoseSupplier;


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

  private MotionMagicVoltage motorMotionMagicController;
  private MotionMagicVoltage motionMagicVolt;

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
  public ShooterPivot(Supplier<Double> joystickSupplier, Supplier<Translation3d> botPoseSupplier) {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.withMotionMagic(new MotionMagicConfigs().withMotionMagicCruiseVelocity(Constants.ShooterPivot.KMotionMagicCruiseVelocity).withMotionMagicAcceleration(Constants.ShooterPivot.KMotionMagicAcceleration));
    motorConfig.withSlot0(new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.ShooterPivot.KShooterPivotKS) 
                .withKV(Constants.ShooterPivot.KShooterPivotKV) 
                .withKA(Constants.ShooterPivot.KShooterPivotKA) 
                .withKG(Constants.ShooterPivot.KShooterPivotKG) 
                .withKP(Constants.ShooterPivot.KShooterPivotKP)
                .withKI(Constants.ShooterPivot.KShooterPivotKI)
                .withKD(Constants.ShooterPivot.KShooterPivotKD));

    motor.getConfigurator().apply(motorConfig);
    motorConfig.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.ShooterPivot.kStatorCurrentLimit).withStatorCurrentLimitEnable(true)); //TODO: need motor stator limit

    //init global variable 
    motorMotionMagicController = new MotionMagicVoltage(0);
    motionMagicVolt = new MotionMagicVoltage(0);

    this.joystickSupplier = joystickSupplier;
    this.botPoseSupplier = botPoseSupplier;

    //configures the motors
    motor.setInverted(Constants.ShooterPivot.kIsShooterPivotInverted);
    motor.setNeutralMode(NeutralModeValue.Brake);
    
    pidController.setTolerance(Constants.ShooterPivot.kAtGoalTolerance);

    motor.setPosition(-0.145);

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

    // ShuffleboardIO.addSlider("shooterPivot kP", 0.0, 0.01, Constants.ShooterPivot.kPID.kP);
    // ShuffleboardIO.addSlider("shooterPivot kD", 0.0, 0.001, 0.0);
    ShuffleboardIO.addSlider("kHorizontalAngle ShooterPivot [SP]", 5.0, 90.0, Constants.ShooterPivot.kTargetAngleStowed);

    // ShuffleboardIO.addSlider("kStartingOffsetAngleDeg [SP]", -5.0, 5, 0.0);
    ShuffleboardIO.addSlider("kTargetAngleAmp [SP]", 60.0, 120, Constants.ShooterPivot.kTargetAngleAmp);
  }

  @Override
  public void periodic() {
    debug();
    motionmagicPeriodic();
  }

  /**
   * @return Angle of the shooterPivot in radians, CW+, CCW-
   */

  public double getShooterPivotAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.ShooterPivot.kShooterPivotGearRatio);

    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }

    return deg;
  }

  /**
   * Gets the pivot speed
   * @param targetAngle Uses the targetAngle to get a set pivot speed.
   * @return
   */

  public void setState(ShooterPivotStates state) {
    this.state = state;
  }

  public ShooterPivotStates getState() {
      return state;
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    motor.setNeutralMode(neutralModeValue);
  }


  public void debug() {
    SmartDashboard.putNumber("Angle in Degrees [SP]", getShooterPivotAngle());
    SmartDashboard.putNumber("Shot Target Angle [SP]", Util.getTargetShotAngleDeg(botPoseSupplier.get(), Util.getSpeakerTargetBasedOnAllianceColor()));
    SmartDashboard.putNumber("Position [SP]", getShooterPivotAngle() / 360);
    SmartDashboard.putString("State [SP]", state.toString());


    //     SmartDashboard.putNumber("ShooterPivot Angle [SP]", getShooterPivotAngle());
    // SmartDashboard.putBoolean("Shooter Pivot atGoal [SP]", atGoal());
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


  /**call this function to run periodic with motion magic */
  private void motionmagicPeriodic() {
    //init default to put in dashboard
    switch (state) {
      case NONE:
      motor.set(0);
        break;
    
       case STOWED:
        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleStowed);

        motor.setControl(motionMagicVolt);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case EJECT_NOTE:
        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleEject);

        motor.setControl(motionMagicVolt);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;


      case INTAKE:
         motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleIntake);

        motor.setControl(motionMagicVolt);
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case INDEXING:
        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleIndexing);

        motor.setControl(motionMagicVolt);        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;
    
      //may not work
      case SPEAKER:
        Translation3d botPose = botPoseSupplier.get();
        System.out.println(botPose.getX() + botPose.getY() + botPose.getZ());
        if (botPose.getX() + botPose.getY() + botPose.getZ() != 0.0) {

          // set angle for pivot target
          // double angleTarget = shooterPivotTable.get(dist); // linear interpolation
          double angleTarget = Util.getTargetShotAngleDeg(botPose, Util.getSpeakerTargetBasedOnAllianceColor());
          double angleTargetAdjusted = Constants.ShooterPivot.kHorizontalAngle - angleTarget;

          if (angleTargetAdjusted < Constants.ShooterPivot.kTargetAngleStowed || angleTargetAdjusted > Constants.ShooterPivot.kTargetAngleAmp) {
            angleTargetAdjusted = Constants.ShooterPivot.kTargetAngleStowed;
          }

          motionMagicVolt = motorMotionMagicController.withPosition(angleTargetAdjusted);
          motor.setControl(motionMagicVolt);  

          // if (angleTargetAdjusted > Constants.ShooterPivot.kMaxAngle) {
          //   motorspeed = getPivotSpeed(Constants.ShooterPivot.kMaxAngle);
          // } else if (angleTargetAdjusted < Constants.ShooterPivot.kTargetAngleStowed) {
          //   motorspeed = getPivotSpeed(Constants.ShooterPivot.kTargetAngleStowed);
          // } else {
          //   motorspeed = getPivotSpeed(angleTargetAdjusted);
          // }

          SmartDashboard.putNumber("Target Angle MECHANISM [SP]", angleTarget);
          SmartDashboard.putNumber("Target Angle ADJUSTED [SP]", angleTargetAdjusted);
        } else {
          SmartDashboard.putNumber("Target Angle MECHANISM [SP]", -1.0);
          SmartDashboard.putNumber("Target Angle ADJUSTED [SP]", -1.0);
          motor.set(0);
        }
        // SmartDashboard.putNumber("Pivot Distance [SP]", dist);
        
        
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
        break;

      case SPEAKER_SUBWOOFER:
        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleSpeakerFromSubwoofer);

        motor.setControl(motionMagicVolt);        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);
      break;

      case SPEAKER_AMP_ZONE:

        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleSpeakerFromAmpZone); 

        motor.setControl(motionMagicVolt);        
        break;

      case SPEAKER_PODIUM:

        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAnglePodium);
        motor.setControl(motionMagicVolt);        

        break;
 
      case CROSSFIELD:

        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleCrossfield); 

        motor.setControl(motionMagicVolt);        
        
        SmartDashboard.putNumber("Shooter Pivot Motor Output [SP]", motorspeed);

        break;
        
      case AMP:
        motionMagicVolt = motorMotionMagicController.withPosition(Constants.ShooterPivot.kTargetAngleAmp);

        
        motor.setControl(motionMagicVolt);        
        
        break;

      case CLIMB: // TODO: add limit
        double speed = joystickSupplier.get() * Constants.ShooterPivot.kManualMultiplier;
        motionMagicVolt = motorMotionMagicController.withPosition(pidController.getSetpoint() + (speed));

      motor.setControl(motionMagicVolt);  
        break;

  }
      SmartDashboard.putString("Shooter Pivot Motion magic ", motionMagicVolt.toString());

}
}
