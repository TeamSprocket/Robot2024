// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrintHello extends Command {
  public PrintHello() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(int i=0; i<=100; i++){
    System.out.println("Hello World!");
    }
  }

  public void end(boolean interrupted) {
  }

   public boolean isFinished() {
    return true;
  }
}
