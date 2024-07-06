
package frc.robot.commands.persistent;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OLDSwerveDrive;
import frc.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTeleop extends Command {
  private final OLDSwerveDrive swerveDrive;
  private final Supplier<Double> xSupplier, ySupplier, tSupplier;
  private final SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

  public DriveTeleop(OLDSwerveDrive swerveDrive, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> tSupplier) {
    this.swerveDrive = swerveDrive;
    this.xSupplier = xSupplier; // PURPOSEFULLY SWITCHED 
    this.ySupplier = ySupplier; // PURPOSEFULLY SWITCHED
    this.tSupplier = tSupplier;
    this.xSlewLimit = new SlewRateLimiter(Constants.OldDrivetrain.kMaxAccel);
    this.ySlewLimit = new SlewRateLimiter(Constants.OldDrivetrain.kMaxAccel);
    this.tSlewLimit = new SlewRateLimiter(Constants.OldDrivetrain.kMaxTurnAccel);
      
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}
  
  
  @Override
  public void execute() {
    double xSpeed = Util.deadband(xSupplier.get(), 0.1);
    xSpeed *= -1;
    double ySpeed = Util.deadband(ySupplier.get(), 0.1);
    ySpeed *= -1;
    double tSpeed = Util.deadband(tSupplier.get(), 0.1);
    // tSpeed *= -1;

      xSpeed = Util.signedSquare(xSpeed);
      ySpeed = Util.signedSquare(ySpeed);
      tSpeed = Util.signedSquare(tSpeed);

    SmartDashboard.putNumber("Squared xSpeed [DTC]", xSpeed);
    
    xSpeed = xSlewLimit.calculate(xSpeed) * Constants.OldDrivetrain.kMaxSpeed;
    ySpeed = ySlewLimit.calculate(ySpeed) * Constants.OldDrivetrain.kMaxSpeed;
    tSpeed = tSlewLimit.calculate(tSpeed) * Constants.OldDrivetrain.kMaxTurnSpeed;

    swerveDrive.updateChassisSpeeds(xSpeed, ySpeed, tSpeed);

    // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, new Rotation2d(swerveDrive.getHeading()));
    // SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
    // swerveDrive.setModuleStates(moduleStates);

    // SmartDashboard.putNumber("heading", swerveDrive.getHeading());
    
  
  }
  

}
