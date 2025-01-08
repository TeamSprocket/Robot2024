package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.SwerveDrive;

public class FollowAprilTag extends Command{
    SwerveDrive swerveDrive;
    public FollowAprilTag(SwerveDrive driveBase) {
        this.swerveDrive = driveBase;
    
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Constants.robotState = RobotState.LOCK_TURN_TO_APRIL_TAG;
        swerveDrive.updateOffsets();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Constants.robotState = RobotState.LOCK_TURN_TO_APRIL_TAG;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Constants.robotState = RobotState.TELEOP;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
                
}
