/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {
  /*
   * 0 - PDP
   * 1 - PCM
   * 10..19 - Talon SRX
   * 20..29 - Talon FX
   * 30..39 - Victor SPX
   * 40..49 - Spark Max
   */
  public static final class Drivetrain {
    public static final int PIGEON_2 = 2;

    public static final int FRONT_LEFT_TALON_D = 21;
    public static final int FRONT_RIGHT_TALON_D = 23;
    public static final int BACK_LEFT_TALON_D = 25;
    public static final int BACK_RIGHT_TALON_D = 27;

    public static final int FRONT_LEFT_TALON_T = 22;
    public static final int FRONT_RIGHT_TALON_T = 24;
    public static final int BACK_LEFT_TALON_T = 26;
    public static final int BACK_RIGHT_TALON_T = 28;

    public static final int FRONT_LEFT_ABS_ENCODER_ID = 31;
    public static final int FRONT_RIGHT_ABS_ENCODER_ID = 32;
    public static final int BACK_LEFT_ABS_ENCODER_ID = 33;
    public static final int BACK_RIGHT_ABS_ENCODER_ID = 34;
  }

  public static final class Elevator{
    public static final int ELEVATOR_LEFT = 11;
    public static final int ELEVATOR_RIGHT = 19;
  }

  public static final class ShooterPivot {
    public static final int WRIST = 13;
  }

  public static final class Intake {
    public static final int ROLL_INTAKE = 14;
    public static final int PIVOT_INTAKE = 15;
  }

  public static final class Shooter {
    public static final int SHOOTER_TOP = 17;
    public static final int SHOOTER_BOTTOM = 12;
    public static final int INDEXER = 16;

    // DIO
    public static final int BEAM_BREAK = 0;
  }

  public static final class Controller {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
    
    public static final int RESET_GYRO_HEADING_BUTTON_ID = 1;

    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW_BUTTON = 7;
    public static final int MENU_BUTTON = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;
  }
}




