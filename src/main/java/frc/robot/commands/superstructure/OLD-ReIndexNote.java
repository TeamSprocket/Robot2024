// package frc.robot.commands.superstructure;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterPivot;
// import frc.robot.subsystems.Intake.IntakeStates;
// import frc.robot.subsystems.Shooter.ShooterStates;
// import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;


// public class ReIndexNote  extends Command {

//   Shooter shooter;
//   ShooterPivot shooterPivot;
//   Timer timer = new Timer();
//   Timer accelTimer = new Timer();
//   boolean hasNote = false;

//   public enum ReIndexNoteStates {
//     START,
//     ROLLFORWARD,
//     WAIT,
//     ROLLBACK,
//     DONE
//   }
//   ReIndexNoteStates state = ReIndexNoteStates.START;

//   public ReIndexNote(Shooter shooter, ShooterPivot shooterPivot) {
//     this.shooter = shooter;
//     this.shooterPivot = shooterPivot;
//   }

//   @Override
//   public void initialize() {
//     this.state = ReIndexNoteStates.START;
    
//     // Constants.robotState = Constants.RobotState.TELEOP_DISABLE_SWERVE;
//     // superstructure.setState(SSStates.INTAKE);

//     hasNote = false;

//     shooter.setState(ShooterStates.STANDBY);
//     shooterPivot.setState(ShooterPivotStates.INTAKE);
    

//     timer.reset();
//     timer.stop();
//   }

//   @Override
//   public void execute() {

//     if (shooter.beamBroken()) {
//       state = ReIndexNoteStates.ROLLFORWARD;
//     }

//     if (state == ReIndexNoteStates.ROLLFORWARD && shooter.hasDetectedNoteShooter()) {
//       timer.start();
//     }

//     if (state == ReIndexNoteStates.ROLLFORWARD && timer.get() > Constants.Superstructure.kIndexerIntakeRollForwardTimeSec) {
//       state = ReIndexNoteStates.WAIT;
//       timer.stop();
//       timer.reset();
//       timer.start();
//     }

//     if (state == ReIndexNoteStates.WAIT && timer.get() > Constants.Superstructure.kRollForwardtoRollBackWaitTime) {
//       timer.stop();
//       timer.reset();
//       state = ReIndexNoteStates.ROLLBACK;
//     }

//     if (state == ReIndexNoteStates.ROLLBACK && shooter.hasNoteRollbackIndexer()) {
//       timer.start();
//     }
    
//     if (state == ReIndexNoteStates.ROLLBACK && timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec) {
//       state = ReIndexNoteStates.DONE;
//     }

//     if (state == ReIndexNoteStates.ROLLFORWARD) {
//       shooter.setState(ShooterStates.INTAKE_ROLLFORWARD);
//     } else if (state == ReIndexNoteStates.WAIT) {
//       shooter.setState(ShooterStates.HOLD_NOTE);
//     } else if (state == ReIndexNoteStates.ROLLBACK) {
//       shooter.setState(ShooterStates.INTAKE_ROLLBACK);
//     }

//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.setState(ShooterStates.STANDBY);    
//     shooterPivot.setState(ShooterPivotStates.STOWED);
//   }

//   @Override
//   public boolean isFinished() {
//     // return timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec;
//     // return shooter.beamBroken();
//     return state == ReIndexNoteStates.DONE;
//   }
// }