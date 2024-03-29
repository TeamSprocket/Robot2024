
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;


public class IntakeNoteManualTimed  extends Command {
  Timer overallTimer = new Timer();
  double duration;

  Intake intake;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Timer timer = new Timer();
  Timer accelTimer = new Timer();
  boolean hasNote = false;

  public IntakeNoteManualTimed(Intake intake, Shooter shooter, ShooterPivot shooterPivot, double duration) {
    this.intake = intake;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void initialize() {
    overallTimer.reset();
    overallTimer.start();


    // Constants.robotState = Constants.RobotState.TELEOP_DISABLE_SWERVE;
    // superstructure.setState(SSStates.INTAKE);

    hasNote = false;

    intake.setState(IntakeStates.INTAKE); // put back
    shooter.setState(ShooterStates.INTAKE_ACCEL);
    shooterPivot.setState(ShooterPivotStates.INTAKE);
    

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

    if (shooter.beamBroken()) { // Note in shooter 
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
    return timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec || overallTimer.get() > duration;
  }
}
