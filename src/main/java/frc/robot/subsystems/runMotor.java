package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunMotor extends SubsystemBase {
 
  private CANSparkMax intake1 = new CANSparkMax(11 , MotorType.kBrushless); //top
  private WPI_TalonFX intake2 = new WPI_TalonFX(1);

  public RunMotor() {
    intake1.setInverted(true); //true for Prototype 1 (flywheel) but false for Prototype 2 (updown)
    intake2.setInverted(false); //false for Prorotype 1
  }
  /**
   * puts values of everything like speed and angle into the smartdashboard
   */

  public void periodic() {
    
  }

  public void run(double output){
    intake1.set(output);
    intake2.set(-output);
  }





  




}











