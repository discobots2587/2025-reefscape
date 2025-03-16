package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.math.geometry.Transform3d;

public class alignReef extends Command {
    private final CoralSubsystem coralSubsystem;
    private final PIDController turnPID = new PIDController(0.02, 0, 0.002); // Tune these values

    public alignReef(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        turnPID.setSetpoint(0); // Align to center (tx = 0)
        turnPID.setTolerance(0.5); // Allowable error in degrees
    }

    @Override
    public void execute() {
        Transform3d cameraL = coralSubsystem.updateCameraPositions(coralSubsystem.cameraL);
        Transform3d cameraR = coralSubsystem.updateCameraPositions(coralSubsystem.cameraR);

        // Use the closest valid camera to align
        double tx = 0;
        if (Math.abs(cameraL.getY()) > 0.001) {
            tx = cameraL.getY();
        } else if (Math.abs(cameraR.getY()) > 0.001) {
            tx = cameraR.getY();
        } else {
            coralSubsystem.setLiftSpeed(0); // No target detected, stop movement
            return;
        }

        double turnSpeed = turnPID.calculate(tx);
        coralSubsystem.setLiftSpeed(turnSpeed); // Adjust rotation based on error
    }

    @Override
    public boolean isFinished() {
        return turnPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setLiftSpeed(0);
    }
}
