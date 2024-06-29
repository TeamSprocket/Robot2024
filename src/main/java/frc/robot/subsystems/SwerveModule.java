package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.util.Conversions;

// import frc.util.ShuffleboardIO;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SwerveModule extends SubsystemBase {
  
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final PIDController turnPIDController; 
  private final CANcoder cancoder;
  private double offset;
  private final boolean turnIsReversed;

  public SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, double offset, boolean driveIsReversed, boolean turnIsReversed, double kPTurnMotor, double kITurnMotor, double kDTurnMotor) {
    this.driveMotor = new TalonFX(driveMotorID, "rio");
    this.turnMotor = new TalonFX(turnMotorID, "rio");
    this.cancoder = new CANcoder(cancoderID, "rio");
    this.offset = offset;
    this.turnIsReversed = turnIsReversed;
    this.driveMotor.setInverted(driveIsReversed); 
    this.turnMotor.setInverted(turnIsReversed);

    turnPIDController = new PIDController(kPTurnMotor, kITurnMotor, kDTurnMotor);
    turnPIDController.enableContinuousInput(-180, 180);
    
    configCancoders();
    configMotors();

    // ShuffleboardIO.addSlider("Swerve PID kP [SD]", 0.0, 0.1, 0);
    // ShuffleboardIO.addSlider("Swerve PID kD [SD]", 0.0, 0.01, 0);
  }

  @Override
  public void periodic() { 
    debug();

    // turnPIDController.setP(ShuffleboardIO.getDouble("Swerve PID kP [SD]"));
    // turnPIDController.setD(ShuffleboardIO.getDouble("Swerve PID kD [SD]"));
    // clearStickyFaults();
  }

  public void setState(SwerveModuleState moduleState) {
    SwerveModuleState state = optimizeState(moduleState);
    // SwerveModuleState state = moduleState;
    
    SmartDashboard.putNumber("Optimized Angle [SM]", moduleState.angle.getDegrees());
    SmartDashboard.putNumber("Drive Speed MPS [SM]", moduleState.speedMetersPerSecond);

    driveMotor.set(state.speedMetersPerSecond);

    // turnMotor.set(0.0, PositionDutyCycle);
    // turnMotor.setControl(state.angle.getRotations());
    // turnMotor.setControl(new PositionDutyCycle(state.angle.getRotations()));
    
    // if (state.speedMetersPerSecond > Constants.Drivetrain.kDrivingMotorDeadband) {
    // turnMotor.set(getTurnPIDOutput(getTurnPosition(), state.angle.getDegrees()));

    double turnSpeed = turnPIDController.calculate(getTurnPosition(), state.angle.getDegrees());
    turnMotor.set(turnSpeed);

    // } else {
      // turnMotor.set(0.0);
    // }
  }

  public SwerveModuleState optimizeState(SwerveModuleState swerveState) {
    // double currentRad = Math.toRadians(getTurnPosition());
    // currentRad %= (Math.PI * 2);
    // if (currentRad < 0) {
    //   currentRad += (Math.PI * 2); 
    // }
    
    return SwerveModuleState.optimize(swerveState, new Rotation2d(getTurnRad()));
  }

  //------Get Methods------

  /**
   * @return Wheel pos in degrees (-180, 180)
   */
  public double getTurnPosition() {
    double deg = Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio);
     deg %= 360;
      if (deg > 180) {
        deg -= (360); 
      } else if (deg < -180) {
        deg += (360);
      }
    return deg;
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
    return cancoder.getAbsolutePosition().getValueAsDouble() * 360; // rotation * 360 = degrees
  }

  public double getDriveVelocity() {
    return driveMotor.getRotorVelocity().getValueAsDouble();
  }

  //------Zero Methods------

  public void zeroTurnMotorABS() {
    //double ticks = Conversions.degreesToFalcon(getCANCoderDegrees(), Constants.Drivetrain.kTurningMotorGearRatio);
    Timer.delay(0.05);
    turnMotor.setPosition((getCANCoderDegrees() / 360) * Drivetrain.kTurningMotorGearRatio);
  }

  public void zeroDriveMotor() {
    driveMotor.setPosition(0.0);
  }

  //------Set Neutral Mode Methods------

  public void setNeutralMode(NeutralModeValue neutralMode) {
    driveMotor.setNeutralMode(neutralMode);
    turnMotor.setNeutralMode(neutralMode);
  }

  public void setNeutralModeDrive(NeutralModeValue neutralMode) {
    driveMotor.setNeutralMode(neutralMode);
    // turnMotor.setNeutralMode(neutralMode);
  }

  public void setNeutralModeTurn(NeutralModeValue neutralMode) {
    // driveMotor.setNeutralMode(neutralMode);
    turnMotor.setNeutralMode(neutralMode);
  }

  //------PID Methods------

  public PIDController getPIDController() {
    return turnPIDController;
  }

  public void updatePIDConstants(double kP, double kI, double kD) {
    turnPIDController.setP(kP);
    turnPIDController.setI(kI);
    turnPIDController.setD(kD);
  }

  //------Configs and Debug Methods------
  
  public void clearStickyFaults() {
    driveMotor.clearStickyFaults();
    turnMotor.clearStickyFaults();
  }

  private void configMotors() {
    TalonFXConfiguration motorConfigDrive = new TalonFXConfiguration();
    TalonFXConfiguration motorConfigTurn = new TalonFXConfiguration();

    driveMotor.getConfigurator().apply(motorConfigDrive);
    turnMotor.getConfigurator().apply(motorConfigTurn);
  }

  private void configCancoders() {
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // makes range of abs sensor to [-0.5, 0.5)
    cancoderConfig.MagnetSensor.MagnetOffset = offset; // configure offset of encoder - read absolute position of encoders without offset and make negative
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // make sure it matches turn degrees of motors
    cancoder.getConfigurator().apply(cancoderConfig);
  }

  private void debug() {
    SmartDashboard.putNumber("ResetTicks", Conversions.degreesToFalcon(cancoder.getAbsolutePosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio));
    SmartDashboard.putNumber("TurnPosDeg2", getTurnPosition());
    SmartDashboard.putNumber("ABSDeg", getCANCoderDegrees());
    SmartDashboard.putNumber("TurnTicks1", turnMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("initialDeg", Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio));
    SmartDashboard.putNumber("Supply Current Drive [SM]", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Supply Current Turn [SM]", turnMotor.getSupplyCurrent().getValueAsDouble());
  }

  // public double getPIDOutput(SwerveModuleState state) {
  //   return turnPIDController.calculate(getTurnPosition(), state.angle.getDegrees());
  // }

  // public double getPIDOutput(double turnAngle, double targetAngle) {
  //   SwerveModuleState state = new SwerveModuleState(1.0, new Rotation2d(Math.toRadians(targetAngle)));
  //   state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(turnAngle)));
  //   return turnPIDController.calculate(turnAngle, state.angle.getDegrees());
  // }

  // public double getTurnPIDOutput(double turnAngle, double targetAngle) {
  //   double kFF = Constants.Drivetrain.kTurnFF;
  //   double error = Math.abs(targetAngle - turnAngle);
  //   if (error < Constants.Drivetrain.kTurnPIDTolerance) {
  //     return turnPIDController.calculate(turnAngle, targetAngle);
  //   }
  //   else {
  //     return kFF;
  //   }
  // }
}
