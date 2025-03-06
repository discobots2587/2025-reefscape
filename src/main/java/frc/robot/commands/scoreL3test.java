package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.NamedCommands;

public class scoreL3test extends Command {
    private Command scoreCommand;

    public scoreL3test() {
        scoreCommand = NamedCommands.getCommand("scoreL3");
    }

    @Override
    public void initialize() {
        if(scoreCommand != null && !scoreCommand.isScheduled()) {
            scoreCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}
