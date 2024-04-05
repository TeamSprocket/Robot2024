
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


public class IntakeNoteManual  extends Command {

  Intake intake;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Timer timer = new Timer();
  Timer accelTimer = new Timer();
  boolean hasNote = false;

  public enum IntakeCommandStates {
    ACCEL,
    INTAKE,
    ROLLFORWARD,
    WAIT,
    ROLLBACK,
    DONE
  }
  IntakeCommandStates state = IntakeCommandStates.ACCEL;

  public IntakeNoteManual(Intake intake, Shooter shooter, ShooterPivot shooterPivot) {
    this.intake = intake;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
  }

  @Override
  public void initialize() {
    this.state = IntakeCommandStates.ACCEL;
    
    // Constants.robotState = Constants.RobotState.TELEOP_DISABLE_SWERVE;
    // superstructure.setState(SSStates.INTAKE);

    hasNote = false;

    intake.setState(IntakeStates.INTAKE);
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
    if (state == IntakeCommandStates.ACCEL && accelTimer.get() > 0.5) {
      state = IntakeCommandStates.INTAKE;
    }

    if (state == IntakeCommandStates.INTAKE && shooter.beamBroken()) {
      state = IntakeCommandStates.ROLLFORWARD;
      intake.setState(IntakeStates.INDEXER);
    }

    if (state == IntakeCommandStates.ROLLFORWARD && shooter.hasDetectedNoteShooter()) {
      timer.start();
    }

    if (state == IntakeCommandStates.ROLLFORWARD && timer.get() > Constants.Superstructure.kIndexerIntakeRollForwardTimeSec) {
      state = IntakeCommandStates.WAIT;
      timer.stop();
      timer.reset();
      timer.start();
    }

    if (state == IntakeCommandStates.WAIT && timer.get() > Constants.Superstructure.kRollForwardtoRollBackWaitTime) {
      timer.stop();
      timer.reset();
      state = IntakeCommandStates.ROLLBACK;
    }

    if (state == IntakeCommandStates.ROLLBACK && shooter.hasNoteRollbackIndexer()) {
      timer.start();
    }
    

    if (state == IntakeCommandStates.ROLLBACK && timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec) {
      state = IntakeCommandStates.DONE;
    }

    if (state == IntakeCommandStates.ACCEL) {
      shooter.setState(ShooterStates.INTAKE_ACCEL);
    } else if (state == IntakeCommandStates.INTAKE) {
      shooter.setState(ShooterStates.INTAKE);
    } else if (state == IntakeCommandStates.ROLLFORWARD) {
      shooter.setState(ShooterStates.INTAKE_ROLLFORWARD);
    } else if (state == IntakeCommandStates.WAIT) {
      shooter.setState(ShooterStates.HOLD_NOTE);
    } else if (state == IntakeCommandStates.ROLLBACK) {
      shooter.setState(ShooterStates.INTAKE_ROLLBACK);
    }

  }

  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeStates.STOWED);
    shooter.setState(ShooterStates.STANDBY);    
    shooterPivot.setState(ShooterPivotStates.STOWED);
  }

  @Override
  public boolean isFinished() {
    // return timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec;
    // return shooter.beamBroken();
    return state == IntakeCommandStates.DONE;
  }
}
