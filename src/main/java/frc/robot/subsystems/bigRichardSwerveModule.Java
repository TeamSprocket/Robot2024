package frc.robot.subsystems;

import java.util.function.Supplier;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
//
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.util.CTREUtils;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;


public class bigRichardSwerveModule extends SubsystemBase {
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private PIDController turnPIDController;
    private CANcoder absEncoder;
    private double absEncoderOffset;
    private boolean isReversed;

    public bigRichardSwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, int absEncoderID, double absEncoderOffsetRad, boolean isTurnedReverse) {
      this.absEncoderOffset = absEncoderOffsetRad;
      absEncoder = new CANcoder(absEncoderID); 

      driveMotor = new TalonFX(driveMotorID);
      turnMotor = new TalonFX(turnMotorID);

      // turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      driveMotor.setInverted(driveMotorIsReversed);
      turnMotor.setInverted(turnMotorIsReversed); 

      this.isReversed = isTurnedReverse;

      turnMotor.setNeutralMode(NeutralModeValue.Coast);
      driveMotor.setNeutralMode(NeutralModeValue.Brake);

      turnPIDController = new PIDController(Constants.bigRichardDriveTrain.kPTurnMotor, Constants.bigRichardDriveTrain.kITurnMotor, Constants.bigRichardDriveTrain.kDTurnMotor);

      turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.MagnetOffset = Math.toDegrees(absEncoderOffsetRad);
      absEncoder.getConfigurator().apply(config);

      config.MagnetSensor.MagnetOffset = -Math.toDegrees(absEncoderOffsetRad);

      absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    

      resetEncoderPos();

  }



    public void setTurnDefaultMode(NeutralModeValue mode) {
      turnMotor.setNeutralMode(mode);
    }
    public void setDriveDefaultMode(NeutralModeValue mode) {
      driveMotor.setNeutralMode(mode);
    }

  public void clearStickyFaults() {
    turnMotor.clearStickyFaults();
    driveMotor.clearStickyFaults();
  }

    
  

  public void setCurrentLimitTurn(double currentLimit) {
    turnMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(1.0));
    turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
  }

  public void setCurrentLimitDrive(double currentLimit) {
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
  }


    public double getDrivePosition() {
      double circum = 2 * (Math.PI) * (Constants.Drivetrain.kWheelDiameterMeters / 2);
      double currentRots = driveMotor.getRotorPosition().getValue() / (2048.0 * Constants.Drivetrain.kDriveMotorGearRatio);
      return currentRots * circum;
    }

    public double getTurnPosition() {
      double posRad =  turnMotor.getRotorPosition().getValue() / Constants.Drivetrain.kTurningMotorGearRatio % 2048 / 2048 * Math.PI * 2;
      if (posRad < 0) {
        posRad += Math.PI * 2;
      }
      return posRad;  

    }

    
    public double getAbsEncoderRad() {
      double angle = absEncoder.getAbsolutePosition().getValueAsDouble();
      angle = Math.toRadians(angle);
      angle = Math.toRadians(angle);
      angle -= absEncoderOffset;
      double rad = Math.abs(angle % (Math.PI * 2.0));
      if (rad > Math.PI) {
        rad = -1.0 * (2.0 * Math.PI - rad);
      }

      angle -= absEncoderOffset;
      return angle;

    }

    public void resetEncoderPos() {
      double tunedAbsEncoderRad = -getAbsEncoderRad();
      turnMotor.getValueAsDouble(tunedAbsEncoderRad / (2.0 * Math.PI) * 2048.0 * Constants.Drivetrain.kTurningMotorGearRatio);
      turnMotor.setPosition((getAbsEncoderRad() / (2 * Math.PI)) * 2048 * Constants.Drivetrain.kTurningMotorGearRatio);
      double absAngle = absEncoder.getAbsolutePosition().getValueAsDouble() % 360.0;
      double absPercent = absAngle / 360.0;
      double absPercentWithRatio = absPercent * Constants.Drivetrain.kTurningMotorGearRatio;
      double encoderPos = absPercentWithRatio * 2048.0;
      turnMotor.setPosition(encoderPos);

    }

    public void zeroTalon() { 
      driveMotor.getRotorPosition();
      turnMotor.setPosition(0);
    }

    public void zeroDrive() {
      driveMotor.setPosition(0);
    }

    public SwerveModuleState optimizeState(SwerveModuleState swerveState) {
      double currentRad = getTurnPosition();
      if (currentRad > Math.PI) {
        currentRad -= (Math.PI * 2); 
      }

      currentRad %= (Math.PI * 2);
      if (currentRad < 0) {
        currentRad += (Math.PI * 2); 
      }
      
      return SwerveModuleState.optimize(swerveState, new Rotation2d(currentRad));
    }


    public void setDesiredState(SwerveModuleState swerveState, boolean isPrecise) {
      SmartDashboard.putNumber("Turn Stator Current", turnMotor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Turn Supply Current", turnMotor.getSupplyCurrent().getValueAsDouble());
      absEncoder.clearStickyFaults();
      driveMotor.clearStickyFaults();
      turnMotor.clearStickyFaults();

      SwerveModuleState state = optimizeState(swerveState);
    

      double fullTargetAngle = state.angle.getRadians();
        if (fullTargetAngle < 0) {
          fullTargetAngle += (Math.PI * 2.0);
        }
      double driveSpd = state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeed;

        if (isPrecise) {
          driveSpd *= Constants.Drivetrain.kPreciseMultiplier;
        }
        
        

      
      if (Math.abs(state.speedMetersPerSecond /  Constants.Drivetrain.kMaxSpeed) < 0.01) {
        stop();
        return;
      }


      if (Constants.Drivetrain.isPrecise) {
        driveSpd /= 3.0;
      }
      driveMotor.set(driveSpd);

      double turnOutput = turnPIDController.calculate(getTurnPosition(),fullTargetAngle);
      turnMotor.set(ControlMode.PercentOutput, turnOutput); 
              
    }

    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
 
}