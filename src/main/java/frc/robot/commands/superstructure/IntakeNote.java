
package frc.robot.commands.superstructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveStates;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.util.Util;

public class IntakeNote extends Command {

  Superstructure superstructure;
  SwerveDrive swerveDrive;
  PIDController pidController;
  Limelight limelight;

  public IntakeNote(Superstructure superstructure, SwerveDrive swerveDrive, Limelight limelight) {
    this.superstructure = superstructure;
    this.swerveDrive = swerveDrive;
    this.limelight = limelight;

    this.pidController = new PIDController(Constants.Drivetrain.kPNoteAlignLL, Constants.Drivetrain.kINoteAlignLL, Constants.Drivetrain.kDNoteAlignLL);
    this.pidController.setSetpoint(0);
  }

  @Override
  public void initialize() {
    SwerveDrive.setState(SwerveDriveStates.TELEOP_DISABLE_SWERVE);
    superstructure.setState(SSStates.INTAKE);
  }

  @Override
  public void execute() {
    // double ySpeed = pidController.calculate(limelight.getIntakeTX()); 
    // ySpeed = Util.minmax(ySpeed, -1.0 * Constants.Drivetrain.kNoteAlignMaxYSpeed, Constants.Drivetrain.kNoteAlignMaxYSpeed);

    // swerveDrive.driveRobotRelative(new ChassisSpeeds(Constants.Drivetrain.kIntakeNoteSpeed, ySpeed, 0));
  }

  @Override
  public void end(boolean interrupted) {
    SwerveDrive.setState(SwerveDriveStates.TELEOP);
    superstructure.setState(SSStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return superstructure.getState() == SSStates.STOWED;
  }
}
