package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralSubsystemConstants.CoralTarget;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToBranch extends Command {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    
    private final DriveSubsystem m_drivetrain;
    private final CoralSubsystem m_coral;
    private final boolean isRightScore;
    
    // Safety timer to stop if we lose the tag for too long
    private final Timer safetyTimer = new Timer();

    public AlignToBranch(DriveSubsystem drivetrain, CoralSubsystem coral, boolean isRight) {
        this.m_drivetrain = drivetrain;
        this.m_coral = coral;
        this.isRightScore = isRight;

        // PID Constants (Tune these!)
        // P=2.0 is a good starting point. If it oscillates, lower to 1.5. If slow, raise to 3.0.
        xController = new PIDController(1.0, 0.0, 0.0); 
        yController = new PIDController(1.0, 0.0, 0.0);
        rotController = new PIDController(0.05, 0.0, 0.0); // Rotation usually needs smaller P

        // Set Tolerances (How close is "good enough"?)
        xController.setTolerance(CoralTarget.kTargetXTol); 
        yController.setTolerance(CoralTarget.kTargetYTol);
        rotController.setTolerance(2.0); // Degrees

        // Set the goal (Setpoints)
        // We want to be at kTargetX distance and 0.0 Y offset (centered)
        xController.setSetpoint(CoralTarget.kTargetX);
        yController.setSetpoint(CoralTarget.kTargetY); 
        rotController.setSetpoint(0.0); // We want 0 degrees rotation error
        
        // Allow rotation to wrap around 180 (so it takes the shortest path)
        rotController.enableContinuousInput(-180, 180);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        safetyTimer.restart();
        
        // Reset PID loops so they don't remember old errors
        xController.reset();
        yController.reset();
        rotController.reset();

        SmartDashboard.putBoolean("AlignToBranch/Running", true);
    }

    @Override
    public void execute() {
        // 1. Get Target Info


        Transform3d targetPos = m_coral.getTargetPos(isRightScore);
        SmartDashboard.putBoolean("AlignToBranch/isRightScore", isRightScore);
        double currentRange = targetPos.getX();
       // currentRange = diff_x;   remove hack
        SmartDashboard.putNumber("AlignToBranch/X",currentRange);
        double currentYOffset = targetPos.getY();
        // currentYOffset = diff_y;  remove hack
        SmartDashboard.putNumber("AlignToBranch/Y",currentYOffset);
        
        // IMPORTANT: Ensure your subsystem returns rotation in the Transform3d
        double currentYawError = Math.toDegrees(targetPos.getRotation().getZ());
        SmartDashboard.putNumber("AlignToBranch/rot",currentYawError); 
        currentYawError=0;//ignore rotaition
        // 2. Safety Check: If data is "empty" (0.0), treat as invalid.
        boolean validTarget = currentRange > 0.01 && currentRange < 4.0; 

        if (validTarget) {
            safetyTimer.reset(); // We saw a tag, reset safety timer

            // 3. Calculate Speeds
            // INVERTED Logic: If range is 2.0 (too far), error is negative, so we must invert to drive forward.
            double xSpeed = -xController.calculate(currentRange);
            double ySpeed = -yController.calculate(currentYOffset);
            double rotSpeed = rotController.calculate(currentYawError);

            // 4. Clamp Speeds (Safety)
            xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
            ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
            rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);

            // 5. Drive (Robot Relative)
            m_drivetrain.drive(xSpeed, ySpeed, rotSpeed, false); 

            // Debug
            SmartDashboard.putNumber("AlignToBranch/X_Speed", xSpeed);
            SmartDashboard.putNumber("AlignToBranch/Y_Speed", ySpeed);
            SmartDashboard.putNumber("AlignToBranch/Range", currentRange);
            SmartDashboard.putNumber("AlignToBranch/YOffset", currentYOffset);
        } else {
            // Target lost? Stop momentarily (or hunt if you prefer)
            m_drivetrain.drive(0, 0, 0, false); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        // STOP the robot immediately when the command ends (button released or target met)
        m_drivetrain.drive(0, 0, 0, false);
        
        SmartDashboard.putBoolean("AlignToBranch/Running", false);
        SmartDashboard.putBoolean("AlignToBranch/Finished", !interrupted);
    }

    @Override
    public boolean isFinished() {
        // FIX: Use AND (&&) so we don't finish until X, Y, AND Rotation are good.
        // FIX: Use AND (&&) so we don't finish until X, Y, AND Rotation are good.
        boolean atX = xController.atSetpoint();
        boolean atY = yController.atSetpoint();
        // boolean atRot = rotController.atSetpoint(); // Optional: enable if rotation is critical

        // Timeout safety: If we haven't seen a tag for 0.5 seconds, just quit.
        if (safetyTimer.hasElapsed(0.5)) {
            return true;
        }

        return atX && atY; 
     // end(true);
    }

}