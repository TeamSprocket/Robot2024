package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Superstructure extends SubsystemBase {
  public static enum SSStates {
    NONE,
    STOWED,
    HANDOFF,
    SPEAKER,
    SPEAKER_HIGH,
    AMP,
    MANUAL
  }
  public SSStates state = SSStates.NONE;
}
