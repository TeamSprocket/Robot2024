// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.macro;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.RobotState;
// import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// public class LockHeadingToSpeaker extends Command {

//   CommandSwerveDrivetrain swerve;
//   Vision limelight;
//   PIDController pidController;

//   public LockHeadingToSpeaker(CommandSwerveDrivetrain swerve, Vision limelight) {
//     this.swerve = swerve;
//     this.limelight = limelight;
//     pidController = new PIDController(0.2, 0, 0);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()
//       .withSpeeds(new ChassisSpeeds(0, 0, pidController.calculate(0, limelight.getXOffset())))
//       .withSteerRequestType(SteerRequestType.MotionMagic)
//     );
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()
//       .withSpeeds(new ChassisSpeeds(0, 0, pidController.calculate(0, limelight.getXOffset())))
//       .withSteerRequestType(SteerRequestType.MotionMagic)
//     );
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return pidController.atSetpoint();
//   }
// }
