package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Shintake.ShintakeStates;
import edu.wpi.first.wpilibj.Timer;

public class ShintakeIntakeNote extends Command{
    Shintake shintake;
    Timer timer = new Timer();

    public ShintakeIntakeNote(Shintake shintake) {
        this.shintake = shintake;
        addRequirements(shintake);
    }

    @Override
    public void initialize() {
        shintake.setState(ShintakeStates.INTAKE_NOTE);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shintake.setState(ShintakeStates.INTAKE_NOTE);
    }

    @Override
    public void end(boolean interrupted) {
        shintake.setState(ShintakeStates.STOWED);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return shintake.hasNote();
    }
}
