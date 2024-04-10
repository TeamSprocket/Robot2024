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
    WAIT_SPEAKER_SUBWOOFER, WAIT_SPEAKER_PODIUM, WAIT_SPEAKER_AMP_ZONE,
    /*WAIT_SPEAKER, WAIT_SPEAKER_HIGH,*/ WAIT_AMP,
    SCORE_SPEAKER_SUBWOOFER, SCORE_SPEAKER_PODIUM, SCORE_SPEAKER_AMP_ZONE,
    /*SCORE_SPEAKER, SCORE_SPEAKER_HIGH,*/ SCORE_AMP,
    CLIMB
  }
  public SSStates state = SSStates.NONE;
  public SSStates lastState = SSStates.NONE;

  Timer timer = new Timer();

  Elevator elevator;
  ShooterPivot shooterPivot;
  Shooter shooter;
  Intake intake;
  Supplier<Pose2d> botPose2dSupplier;

  public Superstructure(Elevator elevator, ShooterPivot shooterPivot, Shooter shooter, Intake intake, Supplier<Pose2d> botPose2dSupplier) {
    this.elevator = elevator;
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
        // elevator.setState(ElevatorStates.NONE); // TODO re-enable
        // shooterPivot.setState(ShooterPivotStates.NONE);
        // shooter.setState(ShooterStates.NONE);
        // intake.setState(IntakeStates.NONE);
      break;
      

      case STOWED:
        elevator.setState(ElevatorStates.STOWED);
        shooterPivot.setState(ShooterPivotStates.STOWED);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case INTAKE:
        // // Reset tolerance timer
        // if (lastState != SSStates.INTAKE) {
        //   timer.reset();
        //   timer.stop();
        // }
        // // Start tolerance timer
        // if (intake.hasDetectedNote()) {
        //   timer.start();
        // }

        // if (shooter.beamBroken()) {
        //   // Note in shooter 
        //   timer.start();
        // } else {
        //   timer.reset();
        //   timer.stop();
        // }

        elevator.setState(ElevatorStates.STOWED);
        shooterPivot.setState(ShooterPivotStates.STOWED);
        shooter.setState(ShooterStates.INTAKE);
        intake.setState(IntakeStates.INTAKE);

        if (timer.get() > Constants.Superstructure.kIndexerIntakeRollBackTimeSec) {
          setState(SSStates.STOWED);
        } 
      break;
      

      // // case WAIT_HANDOFF:
      // // if (allElementsAtGoalNoShooter()) {
      // //   timer.start(); 
      // // } else {
      // //   timer.reset();
      // //   timer.stop();
      // // }

      // //   elevator.setState(ElevatorStates.HANDOFF);
      // //   shooterPivot.setState(ShooterPivotStates.HANDOFF);
      // //   shooter.setState(ShooterStates.HANDOFF);
      // //   intake.setState(IntakeStates.WAIT_HANDOFF);

      // //   if (timer.get() > Constants.Superstructure.kWaitHandoffTimeToleranceSec) {
      // //     setState(SSStates.HANDOFF);
      // //   }

      // // break;


      // case HANDOFF:
      //   if (shooter.beamBroken()) {
      //     // Note in shooter 
      //     timer.start();
      //     // roll intake and shooter
      //   } else {
      //     timer.reset();
      //     timer.stop();
      //   }

      //   // elevator.setState(ElevatorStates.HANDOFF);
      //   // shooterPivot.setState(ShooterPivotStates.HANDOFF);
      //   // shooter.setState(ShooterStates.HANDOFF); // will intake note in resting position
      //   // intake.setState(IntakeStates.HANDOFF); // outtakes note (but stays in normal position)

      //   if (timer.get() >= Constants.Superstructure.kWaitBeambreakTimeToleranceSec) {
      //     setState(SSStates.STOWED);
      //   }
        
      // break;
      

      // case WAIT_SPEAKER:
      //   if (shooterElementsAtGoal()) {
      //     timer.start();
      //   } else {
      //     timer.reset();
      //     timer.stop();
      //   }

      //   elevator.setState(ElevatorStates.SPEAKER);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER);
      //   shooter.setState(ShooterStates.SPINUP);
      //   intake.setState(IntakeStates.STOWED);

      //   // Transitioner
      //   if (timer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      //     setState(SSStates.SCORE_SPEAKER);
      //   }
        
      // break;
      

      // case WAIT_SPEAKER_HIGH:
      // if (shooterElementsAtGoal()) {
      //     timer.start();
      //   } else {
      //     timer.reset();
      //     timer.stop();
      //   }

      //   elevator.setState(ElevatorStates.SPEAKER_HIGH);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER_HIGH);
      //   shooter.setState(ShooterStates.SPINUP);
      //   intake.setState(IntakeStates.STOWED);

      //   // Transitioner
      //   if (timer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      //     setState(SSStates.SCORE_SPEAKER_HIGH);
      //   }
      // break;


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
          setState(SSStates.SCORE_SPEAKER_SUBWOOFER);
        }
        
      break;
      
      

      // case WAIT_SPEAKER_PODIUM:
      //   if (shooterElementsAtGoal() && headingAtGoal()) {
      //     timer.start();
      //   } else {
      //     timer.reset();
      //     timer.stop();
      //   }

      //   // elevator.setState(ElevatorStates.SPEAKER);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
      //   shooter.setState(ShooterStates.SPINUP_PODIUM);
      //   intake.setState(IntakeStates.STOWED);

      //   // Transitioner
      //   if (timer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
      //     setState(SSStates.SCORE_SPEAKER_PODIUM);
      //   }
      // break;
      


      case WAIT_SPEAKER_AMP_ZONE:
        if (shooterElementsAtGoal() && headingAtGoal()) {
          timer.start();
        } else {
          timer.reset();
          timer.stop();
        }

        // elevator.setState(ElevatorStates.SPEAKER);
        shooterPivot.setState(ShooterPivotStates.SPEAKER_TEST);
        shooter.setState(ShooterStates.SPINUP_TEST);
        intake.setState(IntakeStates.STOWED);

        // Transitioner
        if (timer.get() > Constants.Superstructure.kWaitSpeakerTimeToleranceSec) {
          setState(SSStates.SCORE_SPEAKER_AMP_ZONE);
        }
        
      break;
      









      

      case WAIT_AMP:
        elevator.setState(ElevatorStates.AMP);
        shooterPivot.setState(ShooterPivotStates.AMP);
        shooter.setState(ShooterStates.STANDBY);
        intake.setState(IntakeStates.STOWED);
      break;
      

      // case SCORE_SPEAKER:
      //   if (lastState != SSStates.SCORE_SPEAKER) {
      //     timer.reset();
      //     timer.start();
      //   }

      //   elevator.setState(ElevatorStates.SPEAKER);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER);
      //   shooter.setState(ShooterStates.SCORE_SPEAKER);
      //   intake.setState(IntakeStates.STOWED);

      //   if (timer.get() > Constants.Superstructure.kScoreSpeakerTimeToleranceSec) {
      //     setState(SSStates.STOWED);
      //   }
      // break;
      

      // case SCORE_SPEAKER_HIGH:

      // if (lastState != SSStates.SCORE_SPEAKER) {
      //     timer.reset();
      //     timer.start();
      //   }

      //   elevator.setState(ElevatorStates.SPEAKER_HIGH);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER_HIGH);
      //   shooter.setState(ShooterStates.SCORE_SPEAKER_HIGH);
      //   intake.setState(IntakeStates.STOWED);

      //   if (timer.get() > Constants.Superstructure.kScoreSpeakerTimeToleranceSec) {
      //     setState(SSStates.STOWED);
      //   }
      // break;
      

      case SCORE_SPEAKER_SUBWOOFER:
        if (lastState != SSStates.SCORE_SPEAKER_SUBWOOFER) {
          timer.reset();
          timer.start();
        }

        // elevator.setState(ElevatorStates.SPEAKER);
        // shooterPivot.setState(ShooterPivotStates.SPEAKER_SUBWOOFER);
        shooter.setState(ShooterStates.SCORE_SPEAKER_SUBWOOFER);
        intake.setState(IntakeStates.STOWED);

        if (timer.get() > Constants.Superstructure.kScoreSpeakerShootDurationSec) {
          setState(SSStates.STOWED);
        }
      break;

      

      // case SCORE_SPEAKER_PODIUM:
      //   if (lastState != SSStates.SCORE_SPEAKER_PODIUM) {
      //     timer.reset();
      //     timer.start();
      //   }

      //   // elevator.setState(ElevatorStates.SPEAKER);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER_PODIUM);
      //   shooter.setState(ShooterStates.SCORE_SPEAKER_PODIUM);
      //   intake.setState(IntakeStates.STOWED);

      //   if (timer.get() > Constants.Superstructure.kScoreSpeakerShootTimeToleranceSec) {
      //     setState(SSStates.STOWED);
      //   }
      // break;



      // case SCORE_SPEAKER_AMP_ZONE:
      //   if (lastState != SSStates.SCORE_SPEAKER_AMP_ZONE) {
      //     timer.reset();
      //     timer.start();
      //   }

      //   // elevator.setState(ElevatorStates.SPEAKER);
      //   shooterPivot.setState(ShooterPivotStates.SPEAKER_TEST);
      //   shooter.setState(ShooterStates.SCORE_SPEAKER_TEST);
      //   intake.setState(IntakeStates.STOWED);

      //   if (timer.get() > Constants.Superstructure.kScoreSpeakerShootTimeToleranceSec) {
      //     setState(SSStates.STOWED);
      //   }
      // break;

      case SCORE_AMP:
        elevator.setState(ElevatorStates.AMP);
        shooterPivot.setState(ShooterPivotStates.AMP);
        shooter.setState(ShooterStates.SCORE_AMP);
        intake.setState(IntakeStates.STOWED);
      break;
      

      case CLIMB:
        // elevator.setState(ElevatorStates.CLIMB);
        shooterPivot.setState(ShooterPivotStates.CLIMB);
        shooter.setState(ShooterStates.NONE);
        intake.setState(IntakeStates.STOWED);
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
