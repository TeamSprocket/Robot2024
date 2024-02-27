
package frc.robot.subsystems;

import java.util.function.Supplier;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
//
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import frc.util.Conversions;

public class SwerveModule extends SubsystemBase {
  
  
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final PIDController turnPIDController; 
  private final CANcoder cancoder;
  private Supplier<Double> cancoderOffsetDeg;

  public SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, Supplier<Double> cancoderOffsetDeg, boolean driveIsReversed) {
    this.driveMotor = new TalonFX(driveMotorID);
    this.turnMotor = new TalonFX(turnMotorID);
    this.cancoder = new CANcoder(cancoderID);
    this.cancoderOffsetDeg = cancoderOffsetDeg; 
    
    this.driveMotor.setInverted(driveIsReversed); 

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurnMotor, Constants.Drivetrain.kITurnMotor, Constants.Drivetrain.kDTurnMotor);
    turnPIDController.enableContinuousInput(-180, 180);
    turnMotor.setInverted(true);
    
    // cancoder.configFactoryDefault();
    // CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; //Signed_PlusMinus180?
    cancoder.getConfigurator().apply(cancoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ResetTicks", Conversions.degreesToFalcon(cancoder.getAbsolutePosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio));
    SmartDashboard.putNumber("TurnPosDeg2", getTurnPosition());
    SmartDashboard.putNumber("ABSDeg", getCANCoderDegrees());
    SmartDashboard.putNumber("TurnTicks1", turnMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("initialDeg", Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio));
    double deg = Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg -= (deg >  180) ? 360 : 0;
    SmartDashboard.putNumber("Degree", deg);    

    // clearStickyFaults();
    // this.cancoderOffsetDeg = (ShuffleboardPIDTuner.get("CancoderOffsetDegCancoderOffsetDegTEMP"));
  }

  /**
   * @return Wheel pos in degrees (-180, 180)
   */
  public double getTurnPosition() {
    double deg = Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio);
     deg %= 360;
      if (deg > 180) {
        deg -= (360); 
      }
    //deg %= 180;
    return deg;
    //SmartDashboard.putNumber("Degree", deg);
  }



  /**
   * @return Wheel pos in rad [0, 2PI)
   */
  public double getTurnRad() {
    double angle = getTurnPosition();
    angle = Math.toRadians(angle);
    angle = (angle < 0) ? (angle + (2.0 * Math.PI)) : angle;
    return angle;
  }

  /**
   * @return Drive position in meters 
   */
  public double getDrivePosMeters() {
    double motorTicks = driveMotor.getRotorPosition().getValueAsDouble();
    double circum = Constants.Drivetrain.kWheelDiameterMeters * Math.PI;
    double ratio = Constants.Drivetrain.kDriveMotorGearRatio;
    double pos = Conversions.falconToMeters(motorTicks, circum, ratio);
    return pos;
    
  }

  public SwerveModuleState getModuleState() {
    double moduleSpdMPS = Conversions.falconToMPS(driveMotor.getRotorVelocity().getValueAsDouble(), (Constants.Drivetrain.kWheelDiameterMeters * Math.PI), Constants.Drivetrain.kDriveMotorGearRatio);
    return new SwerveModuleState(moduleSpdMPS, new Rotation2d(Math.toRadians(getTurnPosition())));
  }


  public double getCANCoderDegrees() {
    double cancoderRaw = cancoder.getAbsolutePosition().getValueAsDouble();  // 0 to 1
    double cancoderDeg = cancoderRaw * 360.0; // 0 to 360
    double offsetDeg = cancoderDeg - cancoderOffsetDeg.get();
    offsetDeg = (offsetDeg < 0) ? (offsetDeg % 360) + 360 : offsetDeg;
    return offsetDeg;
  }

  public void zeroTurnMotorABS() {
    double ticks = Conversions.degreesToFalcon(getCANCoderDegrees(), Constants.Drivetrain.kTurningMotorGearRatio);
    Timer.delay(0.1);
    turnMotor.setPosition(ticks);
  }

  public void zeroDriveMotor() {
    driveMotor.setPosition(0.0);
  }

  public void setNeutralMode(NeutralModeValue neutralMode) {
    driveMotor.setNeutralMode(neutralMode);
    turnMotor.setNeutralMode(neutralMode);
  }

  public void setState(SwerveModuleState moduleState) {
    SwerveModuleState state = SwerveModuleState.optimize(moduleState, new Rotation2d(Math.toRadians(getTurnPosition()))); //check values, might be jank
    
    // driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond);
    driveMotor.set(state.speedMetersPerSecond);

    // turnMotor.set(ControlMode.PercentOutput, turnPIDController.calculate(getTurnPosition(), state.angle.getDegrees()));
    turnMotor.set(turnPIDController.calculate(getTurnPosition(), state.angle.getDegrees()));

  }
  
  public void clearStickyFaults() {
    driveMotor.clearStickyFaults();
    turnMotor.clearStickyFaults();
  }
  
}
