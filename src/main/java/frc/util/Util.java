/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

/**
 * This class contains miscellaneous utility functions that can be used anywhere
 * in the robot project.
 */
public final class Util {
  /**
   * Return 0 if the value is within -range..range. Otherwise return the value.
   */
  public static double deadband(double value, double range) {
    if(value < range && value > -range)
      return 0;
    else
      return value;
  }

  /**
   * Return 0 if the value is within -range..range. Otherwise return the value.
   */
  public static double deadband(double value, double minValue, double maxValue) {
    if(value > minValue && value < maxValue)
      return 0;
    else
      return value;
  }
  
  /**
   * Limits the value to the given min and max
   */
  public static double minmax(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  /**
   * Returns if the value is in the given range (between the given min and max)
   */
  public static boolean inRange(double value, double min, double max) {
    return (value > min && value < max);
  }


  /**
   * @return Largest numerical parameter 
   */
  public static double max(double... nums) {
    double max = -1.0 * Double.MAX_VALUE;
    for (double num : nums) {
      if (num > max) {
        max = num;
      }
    }

    return max;
  }




  /**
   * Corrects an input based on a polynomial function.
   */
  public static double polynomialCorrect(double input, double... coefficients) {
    double output = 0;
    double term = 1;

    for(double c : coefficients) {
      output += c * term;
      term *= input;
    }

    return output;
  }

  public static double scale(double val, double fromLow, double fromHigh, double toLow, double toHigh) {
    double normalizedVal = (val - fromLow) / (fromHigh - fromLow);
    return normalizedVal * (toHigh - toLow) + toLow;
  }
}
