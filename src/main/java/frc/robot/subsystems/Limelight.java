// package frc.robot.subsystems;

// import java.util.Optional;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.DoubleArraySubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Limelight extends SubsystemBase {

//     private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

//     SwerveDrive swerveDrive;
//     PIDController pidController;
//     double pose[];

//     public Limelight(SwerveDrive swerveDrive) {

//         this.swerveDrive = swerveDrive;
//         this.pidController = new PIDController(Constants.Drivetrain.kLimelightAlignP,
//                 Constants.Drivetrain.kLimelightAlignI, Constants.Drivetrain.kLimelightAlignD);
//         this.pidController.setSetpoint(0);
//     }

//     public void setSpeeds() {

//         double tXOffset = table.getEntry("tx").getDouble(0.0); // why is this tx? ISN'T THAT IN DEGREES BRO WHAT
//         double output = -1 * pidController.calculate(tXOffset);

//         double tYOffset = table.getEntry("ty").getDouble(0.0);
//         // TODO: try to get distance offset!!

//         ChassisSpeeds chassisSpeeds;

//         if (Constants.Drivetrain.kIsFieldOriented) {

//             double headingRad = Math.toRadians(swerveDrive.getHeading());
//             chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, output, 0, new Rotation2d(headingRad));
//         } 
//         else {
//             chassisSpeeds = new ChassisSpeeds(0, output, 0);
//         }

//         SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
//         swerveDrive.setModuleStates(moduleStates);
//         // will only make adjustments for turn?
//     }

//     public void getPipeline() {
//         table.getEntry("getpipe");
//     }

//     public void setAprilTagPipeline() {
//         table.getEntry("pipeline").setNumber(0);
//     }

//     public double getXoffset() {
//         return table.getEntry("tx").getDouble(0.0);
//     }

//     public void changeAlliance() {
//         DriverStation.Alliance color = DriverStation.getAlliance();

//         if (color == DriverStation.Alliance.Blue) {
//             NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
//         } else if (color == DriverStation.Alliance.Red) {
//             NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired");
//         }
//     }

//     public static void pos() {
//         DoubleArraySubscriber x = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
//         double[] pose = x.get();
//         SmartDashboard.putNumber("Limelight x value?", pose[0]);
//         SmartDashboard.putNumber("Limelight y value", pose[1]);
//     }

//     // public Pose2d getPose() {
//     //     DoubleArraySubscriber x = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
//     //     DriverStation.Alliance color = DriverStation.getAlliance();
        
//     //     if (color == DriverStation.Alliance.Blue) {

//     //         x = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
//     //     } else if (color == DriverStation.Alliance.Red) {

//     //         x = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
//     //     }
        
//     //     pose = x.get();
//     //     Pose2d y = new Pose2d(pose[0], pose[1], new Rotation2d(swerveDrive.getHeading())); // TODO: check pose values
//     //     return y;
//     // }

//     // public void stop() {
//     //     swerveDrive.stopModules();
//     // }
// }