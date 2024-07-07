// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.persistent.CommandSwerveDrivetrain;

public class TunerConstants extends SubsystemBase {

  public CommandSwerveDrivetrain DriveTrain;

  private SwerveDrivetrainConstants DrivetrainConstants;
  private SwerveModuleConstantsFactory ConstantCreator;

  private SwerveModuleConstants FrontLeft;
  private SwerveModuleConstants FrontRight;
  private SwerveModuleConstants BackLeft;
  private SwerveModuleConstants BackRight;

  public TunerConstants() {
    DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANbusName(Constants.Drivetrain.kCANbusName)
      .withPigeon2Id(Constants.Drivetrain.kPigeonId)
      .withPigeon2Configs(Constants.Drivetrain.pigeonConfigs);

    ConstantCreator = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(Constants.Drivetrain.kDriveGearRatio)
      .withSteerMotorGearRatio(Constants.Drivetrain.kSteerGearRatio)
      .withWheelRadius(Constants.Drivetrain.kWheelRadiusInches)
      .withSlipCurrent(Constants.Drivetrain.kSlipCurrentA)
      .withSteerMotorGains(Constants.Drivetrain.steerGains)
      .withDriveMotorGains(Constants.Drivetrain.driveGains)
      .withSteerMotorClosedLoopOutput(Constants.Drivetrain.steerClosedLoopOutput)
      .withDriveMotorClosedLoopOutput(Constants.Drivetrain.driveClosedLoopOutput)
      .withSpeedAt12VoltsMps(Constants.Drivetrain.kSpeedAt12VoltsMps)
      .withSteerInertia(Constants.Drivetrain.kSteerInertia)
      .withDriveInertia(Constants.Drivetrain.kDriveInertia)
      .withSteerFrictionVoltage(Constants.Drivetrain.kSteerFrictionVoltage)
      .withDriveFrictionVoltage(Constants.Drivetrain.kDriveFrictionVoltage)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
      .withCouplingGearRatio(Constants.Drivetrain.kCoupleRatio)
      .withDriveMotorInitialConfigs(Constants.Drivetrain.driveInitialConfigs)
      .withSteerMotorInitialConfigs(Constants.Drivetrain.steerInitialConfigs)
      .withCANcoderInitialConfigs(Constants.Drivetrain.cancoderInitialConfigs);

    FrontLeft = ConstantCreator.createModuleConstants(
      Constants.Drivetrain.kFrontLeftSteerMotorId, Constants.Drivetrain.kFrontLeftDriveMotorId, Constants.Drivetrain.kFrontLeftEncoderId, Constants.Drivetrain.kFrontLeftEncoderOffset, Units.inchesToMeters(Constants.Drivetrain.kFrontLeftXPosInches), Units.inchesToMeters(Constants.Drivetrain.kFrontLeftYPosInches), Constants.Drivetrain.kInvertLeftSide)
      .withSteerMotorInverted(Constants.Drivetrain.kFrontLeftSteerInvert);
    FrontRight = ConstantCreator.createModuleConstants(
      Constants.Drivetrain.kFrontRightSteerMotorId, Constants.Drivetrain.kFrontRightDriveMotorId, Constants.Drivetrain.kFrontRightEncoderId, Constants.Drivetrain.kFrontRightEncoderOffset, Units.inchesToMeters(Constants.Drivetrain.kFrontRightXPosInches), Units.inchesToMeters(Constants.Drivetrain.kFrontRightYPosInches), Constants.Drivetrain.kInvertRightSide)
      .withSteerMotorInverted(Constants.Drivetrain.kFrontRightSteerInvert);
    BackLeft = ConstantCreator.createModuleConstants(
      Constants.Drivetrain.kBackLeftSteerMotorId, Constants.Drivetrain.kBackLeftDriveMotorId, Constants.Drivetrain.kBackLeftEncoderId, Constants.Drivetrain.kBackLeftEncoderOffset, Units.inchesToMeters(Constants.Drivetrain.kBackLeftXPosInches), Units.inchesToMeters(Constants.Drivetrain.kBackLeftYPosInches), Constants.Drivetrain.kInvertLeftSide)
      .withSteerMotorInverted(Constants.Drivetrain.kBackLeftSteerInvert);
    BackRight = ConstantCreator.createModuleConstants(
      Constants.Drivetrain.kBackRightSteerMotorId, Constants.Drivetrain.kBackRightDriveMotorId, Constants.Drivetrain.kBackRightEncoderId, Constants.Drivetrain.kBackRightEncoderOffset, Units.inchesToMeters(Constants.Drivetrain.kBackRightXPosInches), Units.inchesToMeters(Constants.Drivetrain.kBackRightYPosInches), Constants.Drivetrain.kInvertRightSide)
      .withSteerMotorInverted(Constants.Drivetrain.kBackRightSteerInvert);

    DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
      FrontRight, BackLeft, BackRight);
  }
}
