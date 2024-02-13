
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunMotor extends SubsystemBase {
 
  private CANSparkMax shooterLeft = new CANSparkMax(11 , MotorType.kBrushless); //top
  private CANSparkMax shooterRight = new CANSparkMax(12, MotorType.kBrushless); //bottom
  private WPI_TalonFX indexer = new WPI_TalonFX(22);
  // private WPI_TalonFX indexer2 = new WPI_TalonFX(23); //change to 23 when done

  /*
   * DEBUG:
   * change device numbers and run together
   * run separately
   * run motors together by setting a set speed (no slider)
   * change sparkmax ports and try testing again
  */

  public RunMotor() {
    // POWER CYCLE IF NOT WORK also set to 0.462 for green wheel prototype
    shooterRight.setInverted(true); //true for Prototype 1 (flywheel) but false for Prototype 2 (updown)
    shooterLeft.setInverted(true); //false for Prorotype 1
  }

  public void periodic() {
    // SmartDashboard.putNumber("Shooter Motor RPS", (shooterLeft.getEncoder().getVelocity() / 60.0));
  }

  public void run(double output, double indexerPower){
    shooterLeft.set(output);
    shooterRight.set(output);
    indexer.set(-indexerPower);
    // indexer2.set(-indexerPower);
  }





  




}











