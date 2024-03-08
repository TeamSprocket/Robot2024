
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunMotor extends SubsystemBase {
 
  // private CANSparkMax shooterLeft = new CANSparkMax(11 , MotorType.kBrushless); //top
  // private CANSparkMax shooterRight = new CANSparkMax(12, MotorType.kBrushless); //bottom
  // private TalonFX indexer = new TalonFX(10);
  // private TalonFX indexer2 = new TalonFX(23); //change to 23 when done

  private TalonFX intake = new TalonFX(0);
  private TalonFX pivot1 = new TalonFX(1);
  private TalonFX pivot2 = new TalonFX(2);

  /*
   * DEBUG:
   * change device numbers and run together
   * run separately
   * run motors together by setting a set speed (no slider)
   * change sparkmax ports and try testing again
  */

  public RunMotor() {
    // POWER CYCLE IF NOT WORK also set to 0.462 for green wheel prototype
    // shooterRight.setInverted(true); //true for Prototype 1 (flywheel) but false for Prototype 2 (updown)
    // shooterLeft.setInverted(true); //false for Prorotype 1
  }

  public void periodic() {
    // SmartDashboard.putNumber("Indexer Motor RPS", (indexer.getMotorOutputVoltage()));
  }

  public void run(double intakeS, double pivot){
    // shooterLeft.set(output);
    // shooterRight.set(output);
    // indexer.set(-indexerPower);
    // indexer2.set(-indexerPower);

    intake.set(intakeS);
    pivot1.set(pivot);
    pivot2.set(-pivot);
  }

}











