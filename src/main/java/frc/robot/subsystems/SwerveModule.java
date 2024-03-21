
// TODO:
// Conversions/units
// PID bounds and continuity 
// Position motor control 




package frc.robot.subsystems;

import java.util.function.Supplier;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
//
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

public class SwerveModule extends SubsystemBase {
  
  
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final PIDController turnPIDController; 
  private final CANcoder cancoder;
  private Supplier<Double> cancoderOffsetDeg;

  private final boolean turnIsReversed;

  public SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, Supplier<Double> cancoderOffsetDeg, boolean driveIsReversed, boolean turnIsReversed, double kPTurnMotor, double kITurnMotor, double kDTurnMotor) {
    this.driveMotor = new TalonFX(driveMotorID, "rio");
    this.turnMotor = new TalonFX(turnMotorID, "rio");
    this.cancoder = new CANcoder(cancoderID, "rio");
    this.cancoderOffsetDeg = cancoderOffsetDeg; 

    this.turnIsReversed = turnIsReversed;
    
    this.driveMotor.setInverted(driveIsReversed); 

    turnPIDController = new PIDController(kPTurnMotor, kITurnMotor, kDTurnMotor);
    turnPIDController.enableContinuousInput(-180, 180);
    this.turnMotor.setInverted(turnIsReversed);
    
    // cancoder.configFactoryDefault();
    // CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; //Signed_PlusMinus180?
    cancoder.getConfigurator().apply(cancoderConfig);

    // ShuffleboardPIDTuner.addSlider("Swerve PID kP [SD]", 0.0, 0.1, 0);
    // ShuffleboardPIDTuner.addSlider("Swerve PID kD [SD]", 0.0, 0.01, 0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Supply Current Drive [SM]", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Supply Current Turn [SM]", turnMotor.getSupplyCurrent().getValueAsDouble());
    

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ResetTicks", Conversions.degreesToFalcon(cancoder.getAbsolutePosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio));
    SmartDashboard.putNumber("TurnPosDeg2", getTurnPosition());
    SmartDashboard.putNumber("ABSDeg", getCANCoderDegrees());
    SmartDashboard.putNumber("TurnTicks1", turnMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("initialDeg", Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio));
    // double deg = Conversions.falconToDegrees(turnMotor.getRotorPosition().getValueAsDouble(), Constants.Drivetrain.kTurningMotorGearRatio);
    // deg -= (deg >  180) ? 360 : 0;
    // SmartDashboard.putNumber("Degree", deg);  
    
    
    // turnPIDController.setP(ShuffleboardPIDTuner.get("Swerve PID kP [SD]"));
    // turnPIDController.setD(ShuffleboardPIDTuner.get("Swerve PID kD [SD]"));
    // WHY no work

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
      } else if (deg < -180) {
        deg += (360);
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
    Timer.delay(0.05);
    turnMotor.setPosition(ticks);
  }

  public void zeroDriveMotor() {
    driveMotor.setPosition(0.0);
  }

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


  public void setState(SwerveModuleState moduleState) {
    SwerveModuleState state = optimizeState(moduleState); //check values, might be jank
    // SwerveModuleState state = moduleState;
    SmartDashboard.putNumber("Optimized Angle [SM]", moduleState.angle.getDegrees());
    
    SmartDashboard.putNumber("Drive Speed MPS [SM]", moduleState.speedMetersPerSecond);
    driveMotor.set(state.speedMetersPerSecond);

    // turnMotor.set(0.0, PositionDutyCycle);
    // turnMotor.setControl(state.angle.getRotations());
    // turnMotor.setControl(new PositionDutyCycle(state.angle.getRotations()));
    
    
    // if (state.speedMetersPerSecond > Constants.Drivetrain.kDrivingMotorDeadband) {
      turnMotor.set( turnPIDController.calculate(getTurnPosition(), state.angle.getDegrees()));
    // } else {
      // turnMotor.set(0.0);
    // }
  }

  public SwerveModuleState optimizeState(SwerveModuleState swerveState) {
    double currentRad = Math.toRadians(getTurnPosition());
    // if (currentRad > Math.PI) {
    //   currentRad -= (Math.PI * 2); 
    // }

    currentRad %= (Math.PI * 2);
    if (currentRad < 0) {
      currentRad += (Math.PI * 2); 
    }
    
    return SwerveModuleState.optimize(swerveState, new Rotation2d(currentRad));
  }


  public double getPIDOutput(SwerveModuleState state) {
    return turnPIDController.calculate(getTurnPosition(), state.angle.getDegrees());
  }

  public double getPIDOutput(double turnAngle, double targetAngle) {
    SwerveModuleState state = new SwerveModuleState(1.0, new Rotation2d(Math.toRadians(targetAngle)));
    state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(turnAngle)));
    return turnPIDController.calculate(turnAngle, state.angle.getDegrees());
  }

  public PIDController getPIDController() {
    return turnPIDController;
  }

  public void updatePIDConstants(double kP, double kI, double kD) {
    turnPIDController.setP(kP);
    turnPIDController.setI(kI);
    turnPIDController.setD(kD);
  }

  public double getDriveVelocity() {
    return driveMotor.getRotorVelocity().getValueAsDouble();
  }

  
  public void clearStickyFaults() {
    driveMotor.clearStickyFaults();
    turnMotor.clearStickyFaults();
  }
  
}
