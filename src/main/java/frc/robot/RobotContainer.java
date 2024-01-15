// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap.Wrist;
// import frc.robot.commands.persistent.DriveTeleop;
import frc.robot.subsystems.runMotor;



public class RobotContainer {

  private final runMotor motor = new runMotor(46, 47);


  public RobotContainer() {
    configureBindings();
    
  }

  public void initAutons() {

    
  }



  
  public void configureBindings() {
    // motor.setDefaultCommand(new DriveTeleop());
		
  


}
}
