// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.OLDSwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreloadtoMidlineBlue extends SequentialCommandGroup {
  /** Creates a new PreloadtoMidlineBlue. */
  public PreloadtoMidlineBlue(OLDSwerveDrive swerveDrive, Intake intake, ShooterPivot shooterPivot, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new ScoreSpeakerSubwooferShootTimed(shooter, intake, shooterPivot), // check if there is a better way to do this
        new Backup(swerveDrive, -3.48, 0), // make sure this translates horizontally
        new Backup(swerveDrive, 0, -7.16),
        new IntakeNoteManualTimed(intake, shooter, shooterPivot, 3)
      )
    );
  }
}
