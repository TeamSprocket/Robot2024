package frc.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Conversions {
    /**
     * @param counts Falcon Position Counts in Rots
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts / gearRatio * 360;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts in Rots
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / 360 * gearRatio;
    }


    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * 60.0;        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }


    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    
    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts / gearRatio * circumference; 
    }



    public static double inchesToMeters(double inches) {
        return (inches * 0.0254);
    }









    // CRESCENDO specific methods

    public static double poseToDistance(Translation2d botPose, Translation3d targetPoint) {
        double x1 = botPose.getX();
        double y1 = botPose.getY();
        double x2 = targetPoint.getX();
        double y2 = targetPoint.getY();

        double dist = Math.sqrt(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2));
        return dist;
    }

    /**
     * @return Target heading in radians
     */
    public static double poseToTargetHeadingRad(Translation2d botPose, Translation3d targetPoint) {
        double x1 = botPose.getX();
        double y1 = botPose.getY();
        double x2 = targetPoint.getX();
        double y2 = targetPoint.getY();

        double targetHeading = Math.atan((y1 - y2) / (x1 - x2)) + 180;
        targetHeading = Math.toRadians(targetHeading);

        return targetHeading;
    }





}