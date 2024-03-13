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

public class bigRichardSwerveModule extends SubsystemBase {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX turnMotor;
    private PIDController turnPIDController;
    private CANCoder absEncoder;
    private double absEncoderOffset;
    private boolean isReversed;

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, int absEncoderID, double absEncoderOffsetRad, boolean isTurnedReverse
  ) {
    this.absEncoderOffsetRad = absEncoderOffsetRad;
    absEncoder = new CANCoder(absEncoderID); 

    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonFX(turnMotorID);

    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    driveMotor.setInverted(driveMotorIsReversed);
    turnMotor.setInverted(turnMotorIsReversed); 

    this.isTurnedReverse = isTurnedReverse;

    turnMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurnMotor, Constants.Drivetrain.kITurnMotor, Constants.Drivetrain.kDTurnMotor);

    turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

    absEncoder.configMagnetOffset(Math.toDegrees(absEncoderOffsetRad));
    absEncoder.configMagnetOffset(-Math.toDegrees(absEncoderOffsetRad));
    absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    

    resetEncoderPos();

  }

    public void setTurnDefaultMode(NeutralMode mode) {
      turnMotor.setNeutralMode(mode);
    }
    public void setDriveDefaultMode(NeutralMode mode) {
      driveMotor.setNeutralMode(mode);
    }

  public void clearStickyFaults() {
    turnMotor.clearStickyFaults();
    driveMotor.clearStickyFaults();
  }

    
  

  public void setCurrentLimitTurn(double currentLimit) {
    turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
    turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
  }

  public void setCurrentLimitDrive(double currentLimit) {
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
  }


    public double getDrivePosition() {
      double circum = 2 * (Math.PI) * (Constants.Drivetrain.kWheelDiameterMeters / 2);
      double currentRots = driveMotor.getSelectedSensorPosition() / (2048.0 * Constants.Drivetrain.kDriveMotorGearRatio);
      return currentRots * circum;
    }

    public double getTurnPosition() {
      double posRad =  turnMotor.getSelectedSensorPosition() / Constants.Drivetrain.kTurningMotorGearRatio % 2048 / 2048 * Math.PI * 2;
      if (posRad < 0) {
        posRad += Math.PI * 2;
      }
      return posRad;  

    }

    
    public double getAbsEncoderRad() {
      double angle = absEncoder.getAbsolutePosition();
      angle = Math.toRadians(angle);
      angle = Math.toRadians(angle);
      angle -= absEncoderOffsetRad;
      double rad = Math.abs(angle % (Math.PI * 2.0));
      if (rad > Math.PI) {
        rad = -1.0 * (2.0 * Math.PI - rad);
      }

      angle -= absEncoderOffsetRad;
      return rad;
      return angle;

    }

    public void resetEncoderPos() {
      double tunedAbsEncoderRad = -getAbsEncoderRad();
      turnMotor.setSelectedSensorPosition(tunedAbsEncoderRad / (2.0 * Math.PI) * 2048.0 * Constants.Drivetrain.kTurningMotorGearRatio);
      turnMotor.setSelectedSensorPosition((getAbsEncoderRad() / (2 * Math.PI)) * 2048 * Constants.Drivetrain.kTurningMotorGearRatio);
      double absAngle = absEncoder.getAbsolutePosition() % 360.0;
      double absPercent = absAngle / 360.0;
      double absPercentWithRatio = absPercent * Constants.Drivetrain.kTurningMotorGearRatio;
      double encoderPos = absPercentWithRatio * 2048.0;
      turnMotor.setSelectedSensorPosition(encoderPos);

    }

    public void zeroTalon() { 
      driveMotor.setSelectedSensorPosition(0);
      turnMotor.setSelectedSensorPosition(0);
    }

    public void zeroDrive() {
      driveMotor.setSelectedSensorPosition(0);
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
      SmartDashboard.putNumber("Turn Stator Current", turnMotor.getStatorCurrent());
      SmartDashboard.putNumber("Turn Supply Current", turnMotor.getSupplyCurrent());
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