
package frc.robot.commands.superstructure;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Superstructure.SSStates;

public class IntakeNote extends Command {

  Superstructure superstructure;
  SwerveDrive swerveDrive;

  public IntakeNote(Superstructure superstructure, SwerveDrive swerveDrive) {
    this.superstructure = superstructure;
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    Constants.robotState = Constants.RobotState.TELEOP_DISABLE_SWERVE;
    superstructure.setState(SSStates.INTAKE);
  }

  @Override
  public void execute() {
    swerveDrive.driveRobotRelative(new ChassisSpeeds(Constants.Drivetrain.kIntakeNoteSpeed, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    Constants.robotState = Constants.RobotState.TELEOP;
    superstructure.setState(SSStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    return superstructure.getState() == SSStates.STOWED;
  }
}
