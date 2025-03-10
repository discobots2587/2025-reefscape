package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.NamedCommands;

public class scoreL3test extends Command {
    private Command scoreCommand;
    private boolean isScoreCommandFinished;

    public scoreL3test() {
        scoreCommand = NamedCommands.getCommand("scoreL3");
        isScoreCommandFinished = false;
    }

    @Override
    public void initialize() {
        if (scoreCommand != null && !scoreCommand.isScheduled()) {
            scoreCommand.schedule();
            isScoreCommandFinished = false;
        }
    }

    @Override
    public void execute() {
        // Check if the scoreCommand has finished
        if (scoreCommand != null && scoreCommand.isFinished()) {
            isScoreCommandFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the scoreCommand has completed
        return isScoreCommandFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // If the command is interrupted, cancel the scoreCommand
        if (scoreCommand != null) {
            scoreCommand.cancel();
        }
    }
}
