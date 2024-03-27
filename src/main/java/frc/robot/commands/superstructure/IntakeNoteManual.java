
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
  Timer accelTimer = new Timer();
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

    intake.setState(IntakeStates.INTAKE); // put back
    shooter.setState(ShooterStates.INTAKE_ACCEL);

    timer.reset();
    timer.stop();

    accelTimer.reset();

    accelTimer.start();
  }

  @Override
  public void execute() {
    // swerveDrive.driveRobotRelative(new ChassisSpeeds(Constants.Drivetrain.kIntakeNoteSpeed, 0, 0));
    if (!hasNote && accelTimer.get() > 0.5) {
      shooter.setState(ShooterStates.INTAKE);
    }

    if (shooter.hasDetectedNote()) { // Note in shooter 
      hasNote = true;
      timer.start();
    } 

    if (hasNote) {
      // intake.setState(IntakeStates.INTAKE_ROLLBACK);
      shooter.setState(ShooterStates.INTAKE_ROLLBACK);
      
    }
    


  }

  @Override
  public void end(boolean interrupted) {
    // Constants.robotState = Constants.RobotState.TELEOP;
    // superstructure.setState(SSStates.STOWED);

    intake.setState(IntakeStates.STOWED); // put back
    shooter.setState(ShooterStates.STANDBY);
    
  }

  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec;
  }
}
