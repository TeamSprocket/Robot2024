// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.persistent.DriveTeleop;
// import frc.robot.commands.persistent.DriveTeleop;
import frc.robot.subsystems.RunMotor;



public class RobotContainer {

  //private final XboxController driver = new XboxController(0);

  private final RunMotor motor = new RunMotor();


  public RobotContainer() {
    //configureBindings();
    
  }
  
  public void configureBindings() {
    motor.setDefaultCommand(new DriveTeleop(motor));
		
  


}
}
