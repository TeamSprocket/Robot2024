package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;


public class AlignNoteIndexing  extends Command {

  Shooter shooter;
  ShooterPivot shooterPivot;
  Timer timer = new Timer();
  Timer accelTimer = new Timer();
  boolean hasNote = false;

  public enum AlignNote {
    START,
    ROLLFORWARD,
    WAIT,
    ROLLBACK,
    DONE
  }
  AlignNote state = AlignNote.START;

  public AlignNoteIndexing(Shooter shooter, ShooterPivot shooterPivot) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void initialize() {
    this.state = AlignNote.START;
    
    // Constants.robotState = Constants.RobotState.TELEOP_DISABLE_SWERVE;
    // superstructure.setState(SSStates.INTAKE);

    hasNote = false;

    shooter.setState(ShooterStates.STANDBY);
    shooterPivot.setState(ShooterPivotStates.INTAKE);
    

    timer.reset();
    timer.stop();
  }

  @Override
  public void execute() {

    if (shooter.beamBroken()) {
      state = AlignNote.ROLLFORWARD;
    }

    if (state == AlignNote.ROLLFORWARD && shooter.hasDetectedNoteShooter()) {
      timer.start();
    }

    if (state == AlignNote.ROLLFORWARD && timer.get() > Constants.Superstructure.kIndexerIntakeRollForwardTimeSec) {
      state = AlignNote.WAIT;
      timer.stop();
      timer.reset();
      timer.start();
    }

    if (state == AlignNote.WAIT && timer.get() > Constants.Superstructure.kRollForwardtoRollBackWaitTime) {
      timer.stop();
      timer.reset();
      state = AlignNote.ROLLBACK;
    }

    if (state == AlignNote.ROLLBACK && shooter.hasNoteRollbackIndexer()) {
      timer.start();
    }
    
    if (state == AlignNote.ROLLBACK && timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec) {
      state = AlignNote.DONE;
    }

    if (state == AlignNote.ROLLFORWARD) {
      shooter.setState(ShooterStates.INTAKE_ROLLFORWARD);
    } else if (state == AlignNote.WAIT) {
      shooter.setState(ShooterStates.HOLD_NOTE);
    } else if (state == AlignNote.ROLLBACK) {
      shooter.setState(ShooterStates.INTAKE_ROLLBACK);
    }

  }

  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterStates.STANDBY);    
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    // return timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec;
    // return shooter.beamBroken();
    return state == AlignNote.DONE;
  }
}
