// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.ScoreSpeakerSubwooferShootTimed;
import frc.robot.commands.macro.WaitTimed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoNothing extends SequentialCommandGroup {
  /** Creates a new OneNoteNoTaxi. */
  public DoNothing() {
    addCommands(
      new WaitTimed(3.473)
    );
  }
}
