// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.common.hardware.VisionLEDMode;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import frc.util.Util;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.filter.MedianFilter;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.SwerveDrive;
// import frc.robot.Constants;

// public class Vision extends SubsystemBase {

//     private PhotonCamera shooterCam = new PhotonCamera("limelight-shooter");
//     private PhotonCamera intakeCam = new PhotonCamera("limelight-intake");

//     private MedianFilter filterY = new MedianFilter(3);
//     private MedianFilter filterX = new MedianFilter(3);
//     private MedianFilter filterIntake = new MedianFilter(5);

//     private ArrayList<Double> distsX = new ArrayList<Double>(Constants.Limelight.kVolatilitySlidingWindowLen);
//     private ArrayList<Double> distsY = new ArrayList<Double>(Constants.Limelight.kVolatilitySlidingWindowLen);



//     public Vision() {
//         for (int i = 0; i < Constants.Limelight.kVolatilitySlidingWindowLen; i++) {
//             distsX.add(0.0);
//             distsY.add(0.0);
//         }




//         shooterCam.setLED(VisionLEDMode.kOff);
//         intakeCam.setLED(VisionLEDMode.kOff);

//         AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//         Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
//         PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, shooterCam, robotToCam);
        

//         photonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
//         Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        
//         return pose;


//         List<PhotonTrackedTarget> targets = shooterCam.getLatestResult().getTargets();
//         for (PhotonTrackedTarget target : targets) {
//             int id = target.getFiducialId();
//             double ambiguity = target.getPoseAmbiguity();
//         }
        

//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Pose X [LL]", shooterCam.getEntry("botpose").getDoubleArray(new double[2])[0]);
//         SmartDashboard.putNumber("Pose Y [LL]", shooterCam.getEntry("botpose").getDoubleArray(new double[2])[1]);
//         SmartDashboard.putNumber("Filtered X [LL]", getTranslation2d().getX());
//         SmartDashboard.putNumber("Filtered Y [LL]", getTranslation2d().getY());
//         SmartDashboard.putNumber("Averaged X [LL]", getTranslation2dWindowAvg(Constants.Limelight.kAverageWindowSize, Constants.Limelight.kAverageWindowBuffer).getX());
//         SmartDashboard.putNumber("Averaged Y [LL]", getTranslation2dWindowAvg(Constants.Limelight.kAverageWindowSize, Constants.Limelight.kAverageWindowBuffer).getY());


//         SmartDashboard.putNumber("Intake Note TX [LL]", getNoteTX());

//         SmartDashboard.putNumber("Pose Volatility [LL]", getPoseVolatility());
//         SmartDashboard.putBoolean("Pose Is Not Volatile [LL]", getIsNotVolatile());


//         // System.out.println(""+shooterCam.getEntry("botpose").getDoubleArray(new double[2])[0]);
//         Translation2d currentPose = getTranslation2d();

//         distsX.add(filterX.calculate(currentPose.getX()));
//         distsX.remove(0);

//         distsY.add(filterY.calculate(currentPose.getY()));
//         distsY.remove(0);
        
//     }

//     public Translation2d getTranslation2d() {
//         double[] botPose;

//         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
//             botPose = shooterCam.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
//         }
//         else {
//             botPose = shooterCam.getEntry("botpose_wpired").getDoubleArray(new double[2]);
//         }

//         double xPos = botPose[0];
//         double yPos = botPose[1];


//         // double filteredX = filterX.calculate(xPos);
//         // double filteredY = filterY.calculate(yPos);

//         // return (new Translation2d(filteredX, filteredY));
//         return new Translation2d(xPos, yPos);
//     }

//     /**
//      * @return Average bot pose Translation2d in the entire read window 
//      */
//     public Translation2d getTranslation2dWindowAvg() {
//         return new Translation2d(
//             Util.average(distsX),
//             Util.average(distsY)
//         );
//     }

//     /**
//      * @param numTerms number of terms included in the window, will the most recent numTerms terms 
//      * @return Average bot pose Translation2d in the window  
//      */
//     public Translation2d getTranslation2dWindowAvg(int numTerms) {
//         int initialXIndex = distsX.size() - 1 - numTerms;
//         int finalXIndex = distsX.size() - 1;
//         int initialYIndex = distsY.size() - 1 - numTerms;
//         int finalYIndex = distsY.size() - 1;
//         return new Translation2d(
//             Util.average(distsX.subList(initialXIndex, finalXIndex)),
//             Util.average(distsY.subList(initialYIndex, finalYIndex))
//         );
//     }

//     /**
//      * @param numTerms number of terms included in the window, will the most recent numTerms terms 
//      * @param numEndBuffer number of buffer terms not to include at the end of sliding window
//      * @return Average bot pose Translation2d in the window  
//      */
//     public Translation2d getTranslation2dWindowAvg(int numTerms, int numEndBuffer) {
//         int initialXIndex = distsX.size() - 1 - numTerms - numEndBuffer;
//         int finalXIndex = distsX.size() - 1 - numEndBuffer;
//         int initialYIndex = distsY.size() - 1 - numTerms - numEndBuffer;
//         int finalYIndex = distsY.size() - 1 - numEndBuffer;
//         return new Translation2d(
//             Util.average(distsX.subList(initialXIndex, finalXIndex)),
//             Util.average(distsY.subList(initialYIndex, finalYIndex))
//         );
//     }





//     public double getNoteTX() {
//         double noteTX = intakeCam.getEntry("tx").getDouble(0.0);
//         noteTX = filterIntake.calculate(noteTX);
//         return noteTX;
//     }

//     /**
//      * @return Returns whether volatility is below threshold AND apriltags are detected 
//      */
//     public boolean getIsNotVolatile() {
//         Translation2d currentPose = getTranslation2d();
//         return getPoseVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold && currentPose.getX() != 0.0 && currentPose.getX() != 0.0;
//     }

//     /**
//      * @return Returns volatility of pose reading 
//      */
//     public double getPoseVolatility() {
//         double volatilityX = getPoseVolatilityAxis(distsX);
//         double volatilityY = getPoseVolatilityAxis(distsY);

//         return Math.max(volatilityX, volatilityY);
//     }

//     private double getPoseVolatilityAxis(ArrayList<Double> distances) {
//         double average = Util.average(distances);

//         double totalDeviation = 0;
//         for (double dist : distances) {
//             totalDeviation += Math.abs(dist - average);
//         }

//         return totalDeviation;
//     }

    
//     // public double getVolatilityAxis(double average, ArrayList llOdometry){
//     //     double volatility = 0.00;
//     //     for(int i = 0; i < 50; i++){
//     //         volatility += Math.abs(average - (double)llOdometry.get(i));
//     //     }
//     //     return volatility;
//     // }

//     // /**
//     //  * @return Highest axial volatility reading
//     //  */
//     // public double getOverallVolatility() {
//     //     double volatilityX = getVolatilityAxis(averageX, llOdometryX);
//     //     double volatilityY = getVolatilityAxis(averageY, llOdometryY);
//     //     double overallVolatility = Util.max(volatilityX, volatilityY);
//     //     return overallVolatility;
//     // }


//     // public boolean getIsNotVolatile() {
//     //     return getOverallVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold;
//     // }





// }
