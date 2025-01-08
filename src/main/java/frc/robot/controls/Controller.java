
package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


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





}
