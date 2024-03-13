package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Shintake.ShintakeStates;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
//When the command is activated, it activates WaitAmp which moves the pivot to the setposition and then at that position it switches to ScoreAmp so that
//it can score in the amp. After a time delay, it goes back to stowed.
public class ShintakeScoreAmp extends InstantCommand{
    Shintake shintake;
    Timer timer = new Timer();
    boolean atSpeed = false;

    public ShintakeScoreAmp(Shintake shintake) {
        this.shintake = shintake;
        addRequirements(shintake);
    }

    @Override
    public void initialize() {
        shintake.setState(ShintakeStates.WAIT_AMP);
        timer.reset();
        timer.stop();
        // timer.start();
    }

    @Override
    public void execute() {
        if (Math.abs(Constants.Shintake.kRollSpeedScoreAmp - shintake.getShintakeMotor().getRotorPosition().getValueAsDouble()) < Math.abs(Constants.Shintake.kMotorShintakeTolerance)) {
            // shintake.setState(ShintakeStates.WAIT_AMP);
            atSpeed = true;
            timer.start();
        }
        if (atSpeed) {
            shintake.setState(ShintakeStates.SCORE_AMP);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shintake.setState(ShintakeStates.STOWED);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > Constants.Shintake.kDurationScoreAmpSec;
    }
}
