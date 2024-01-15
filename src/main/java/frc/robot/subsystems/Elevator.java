package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX motorLeft = new WPI_TalonFX(0);
  private final WPI_TalonFX motorRight = new WPI_TalonFX(0);
  /** Creates a new elevator. */
  public Elevator() {
    motorRight.follow(motorLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void goingup(){
    motorLeft.set(0.5);
  }
  public void goingdown(){
    motorLeft.set(-0.5);
  }
  public void stop(){
    motorLeft.set(0);

  }

}



