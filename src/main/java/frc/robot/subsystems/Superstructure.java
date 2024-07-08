package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.ShooterPivot.ShooterPivotStates;
import frc.util.Conversions;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Intake.IntakeStates;

public class Superstructure extends SubsystemBase {
  public static enum SSStates {
    NONE,
    STOWED,
    INTAKE,
    WAIT_SPEAKER_SUBWOOFER, WAIT_SPEAKER_PODIUM,
    SCORE,
    EJECT_NOTE
  }
  public SSStates state = SSStates.NONE;
  public SSStates lastState = SSStates.NONE;

  Timer timer = new Timer();

  ShooterPivot shooterPivot;
  Shooter shooter;
  Intake intake;
  Supplier<Pose2d> botPose2dSupplier;

  public Superstructure(ShooterPivot shooterPivot, Shooter shooter, Intake intake, Supplier<Pose2d> botPose2dSupplier) {
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intake = intake;
    this.botPose2dSupplier = botPose2dSupplier;

    timer.reset();
  }

  // ELevator: NONE, STOWED, HANDOFF, SPEAKER, SPEAKER_HIGH, AMP, MANUAL
  // ShooterPivot: NONE, STOWED, HANDOFF, SPEAKER, SPEAKER_HIGH, AMP, MANUAL
  // Shooter: NONE, STANDBY, HANDOFF, SPINUP, SCORE_SPEAKER, SCORE_AMP
  // Intake: NONE, STOWED, INTAKE, WAIT_HANDOFF, HANDOFF



  @Override
  public void periodic() {
    switch (state) {
      case NONE:

      break;
      

      case STOWED:
        shooterPivot.setState(ShooterPivotStates.STOWED);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case INTAKE:

        shooterPivot.setState(ShooterPivotStates.STOWED);
        shooter.setState(ShooterStates.INTAKE);
        intake.setState(IntakeStates.INTAKE);

        if (timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec) {
          setState(SSStates.STOWED);
        } 
      break;

      case WAIT_SPEAKER_SUBWOOFER:
        if (shooterElementsAtGoal() && headingAtGoal()) {
          timer.start();
        } else {
          timer.reset();
          timer.stop();
        }

        // elevator.setState(ElevatorStates.SPEAKER);
        // shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
        shooter.setState(ShooterStates.SPINUP_SUBWOOFER);
        intake.setState(IntakeStates.STOWED);

        // Transitioner
        if (timer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
          setState(SSStates.SCORE);
        }
        
      break;
      
      case WAIT_SPEAKER_PODIUM:
        if (shooterElementsAtGoal() && headingAtGoal()) {
          timer.start();
        } else {
          timer.reset();
          timer.stop();
        }

        // elevator.setState(ElevatorStates.SPEAKER);
        shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
        shooter.setState(ShooterStates.SPINUP_PODIUM);
        intake.setState(IntakeStates.STOWED);

        // Transitioner
        if (timer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
          setState(SSStates.SCORE);
        }
      break;
      
      case SCORE:
        if (lastState != SSStates.SCORE) {
          timer.reset();
          timer.start();
        }

        // elevator.setState(ElevatorStates.SPEAKER);
        // shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
        // shooter.setState(ShooterStates.SCORE_SPEAKER_SUBWOOFER);
        intake.setState(IntakeStates.STOWED);

        if (timer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec) {
          setState(SSStates.STOWED);
        }
      break;

      case EJECT_NOTE:
      break;
      
      }

    lastState = state;
  }




  // Methods

  /**
   * Sets the superstructure target state
   * @param state Target state
   */
  public void setState(SSStates state) {
      this.state = state;
  }

  public SSStates getState() {
      return state;
  }



  /**
   * @return SHOOTER INCLUDED, if all elements are within tolerance of their goals
   */
  public boolean allElementsAtGoal() {
    // return elevator.atGoal() && shooterPivot.atGoal() && intake.atGoal() && shooter.atGoalShooter();
    return true;
  }


  /**
   * @return SHOOTER NOT INCLUDED, if all elements are within tolerance of their goals
   */
  public boolean allElementsAtGoalNoShooter() {
    // return elevator.atGoal() && shooterPivot.atGoal() && intake.atGoal();
    return true;
  }


  /**
   * @return Just shooter and shooterPivot within tolerance of their goals 
   */
  public boolean shooterElementsAtGoal() {
    // return elevator.atGoal() && shooterPivot.atGoal() && shooter.atGoalShooter();
    return true;
  }


  /**
   * @return Heading of bot pointed at correct target
   */
  public boolean headingAtGoal() {
    // double targetHeadingRad = Conversions.poseToTargetHeadingRad(botPose2dSupplier.get().getTranslation(), Constants.ShootingSetpoints.targetPoint);
    // double currentHeadingRad = botPose2dSupplier.get().getRotation().getRadians();
    // double error = Math.abs(targetHeadingRad - currentHeadingRad);
    // return error < Constants.Superstructure.kScoreSpeakerHeadingTolerance;
    return false;
  }







}
