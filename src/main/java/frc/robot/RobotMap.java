/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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
    public static final int FRONT_LEFT_TALON_D = 23;
    public static final int FRONT_RIGHT_TALON_D = 27;
    public static final int BACK_LEFT_TALON_D = 21;
    public static final int BACK_RIGHT_TALON_D = 25;

    public static final int FRONT_LEFT_TALON_T = 24;
    public static final int FRONT_RIGHT_TALON_T = 28;
    public static final int BACK_LEFT_TALON_T = 22;
    public static final int BACK_RIGHT_TALON_T = 26;

    public static final int FRONT_LEFT_ABS_ENCODER_ID = 1;
    public static final int FRONT_RIGHT_ABS_ENCODER_ID = 3;
    public static final int BACK_LEFT_ABS_ENCODER_ID = 0;
    public static final int BACK_RIGHT_ABS_ENCODER_ID = 2;

    // public static final int kTurnP = 0;
    // public static final int kTurnI = 0;
    // public static final int kTurnD = 0;
    

    // public static final int FRONT_LEFT_TALON_E = 9;
    // public static final int FRONT_RIGHT_TALON_E = 10;
    // public static final int BACK_LEFT_TALON_E = 11;
    // public static final int BACK_RIGHT_TALON_E = 12;

  }

  public static final class Elevator{
    public static final int ELEVATOR_LEFT = 40;
    public static final int ELEVATOR_RIGHT = 41;
  }

  public static final class Arm{
    public static final int ARM_LEFT = 42;
    public static final int ARM_RIGHT = 43;
  }

  public static final class Wrist{
    public static final int WRIST = 44;
  }

  public static final class Claw{
    public static final int CLAW = 45; // or 46


    //public static final int PISTON_LEFT = 14;
    //public static final int PISTON_RIGHT = 15;

  }
  public static final class LEDStrip {
    public static final int LED = 9;
  }

  public static final class PCH {
    //public static final int PCH_CAN = 1;
    //public static final int PRESSURE_SENSOR_CHANNEL = 0;
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
    public static final int LOGO_LEFT = 7;
    public static final int LOGO_RIGHT = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;
  }
}

















// -=All=-
// DEPORT = RIGHT JOYSTICK
// HOME = X
// RESET = Funny Button

// -=CONES=-
// HIGH = Y
// MID = A
// LOW = LEFT JOYSTICK
// FLOOR = RIGHT BUMPER

// -=CUBES=-
// HIGH = B
// MID = Chair
// LOW = LEFT BUMPER







