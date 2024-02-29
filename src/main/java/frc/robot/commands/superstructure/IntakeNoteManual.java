
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;


public class IntakeNoteManual  extends Command {

  Intake intake;
  Shooter shooter;
  Timer timer = new Timer();
  boolean hasNote = false;

  public IntakeNoteManual(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    // Constants.robotState = Constants.RobotState.TELEOP_DISABLE_SWERVE;
    // superstructure.setState(SSStates.INTAKE);

    hasNote = false;

    intake.setState(IntakeStates.INTAKE);
    shooter.setState(ShooterStates.INTAKE);

    timer.reset();
    timer.stop();
  }

  @Override
  public void execute() {
    // swerveDrive.driveRobotRelative(new ChassisSpeeds(Constants.Drivetrain.kIntakeNoteSpeed, 0, 0));

    if (shooter.hasDetectedNote()) { // Note in shooter 
      hasNote = true;
      timer.start();
    } 

    if (hasNote) {
      shooter.setState(ShooterStates.INTAKE_ROLLBACK);
    }
    


  }

  @Override
  public void end(boolean interrupted) {
    // Constants.robotState = Constants.RobotState.TELEOP;
    // superstructure.setState(SSStates.STOWED);

    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);
    
  }

  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Superstructure.kIndexerIntakeRollbackTimeSec;
  }
}
