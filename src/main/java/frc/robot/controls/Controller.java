
package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Controller extends SubsystemBase {

  CommandXboxController controller;

  boolean isRumbling = false;
  Timer rumbleTimer = new Timer();

  public Controller(int controllerID) {
    this.controller = new CommandXboxController(controllerID);

    this.rumbleTimer.reset();
    this.rumbleTimer.stop();
  }

  @Override
  public void periodic() {
    
  }

  public CommandXboxController getController() {
      return controller;
  }

  public void updateNoteRumbleListener(Supplier<Boolean> beamBrokenSupplier, Supplier<ShooterStates> shooterStateSupplier) {

    boolean beamBroken = beamBrokenSupplier.get();
    ShooterStates shooterState = shooterStateSupplier.get();
    boolean isNewIntakeCycle = true;
    
    // Start rumble
    if (!isRumbling && beamBroken && isNewIntakeCycle && 
        shooterState == ShooterStates.INTAKE || shooterState == ShooterStates.INTAKE_ROLLFORWARD) {

      rumbleTimer.start();
      isRumbling = true;
      isNewIntakeCycle = false;
    }

    // Set rumble  
    if (isRumbling) {
      controller.getHID().setRumble(RumbleType.kBothRumble, Constants.Controller.kHasNoteRumbleIntensity);
    }

    // End rumble 
    if (isRumbling && rumbleTimer.get() > Constants.Controller.kHasNoteRumbleTimeSec) {
      controller.getHID().setRumble(RumbleType.kBothRumble, 0);
      isRumbling = false;

      rumbleTimer.reset();
      rumbleTimer.stop();
    }

    // Note clear, reset rumble 
    if (!isRumbling && !beamBroken && shooterState == ShooterStates.STANDBY) {
      isNewIntakeCycle = true;
    }
  }

  public void stopNoteRumbleListener() {
    controller.getHID().setRumble(RumbleType.kBothRumble, 0);
  }


}
