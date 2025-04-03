// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;




public class AlignToBranch extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore = true;
    private Timer dontSeeTagTimer, stopTimer, idleTimer;
    private DriveSubsystem m_drivetrain;
    private CoralSubsystem m_coral;
    private double tagID = -1;
    

  public AlignToBranch(DriveSubsystem m_drivetrain, CoralSubsystem coral, boolean is_right, int tag) {
    double p_x = 1,p_y = 2.0, p_r = 0.1625;
    xController = new PIDController(p_x, 0.0, 0);  // Vertical movement
    yController = new PIDController(p_y, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(p_r, 0, 0);  // Rotation
    this.m_drivetrain = m_drivetrain;
    this.m_coral = coral;
    this.tagID = tag;
    this.isRightScore = is_right;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.idleTimer = new Timer();
    this.idleTimer.start();


    
    xController.setSetpoint(Constants.CoralSubsystemConstants.CoralTarget.kTargetX);
    xController.setTolerance(Constants.CoralSubsystemConstants.CoralTarget.kTargetXTol);

    yController.setSetpoint(Constants.CoralSubsystemConstants.CoralTarget.kTargetY);
    yController.setTolerance(Constants.CoralSubsystemConstants.CoralTarget.kTargetYTol);

    tagID = -1;
  }

  @Override
  public void execute() { 
    Transform3d target_pos = m_coral.getTargetPos(isRightScore); 
    double tX = target_pos.getX();
    double tY = target_pos.getY();
    double path_x = target_pos.getX() - Constants.CoralSubsystemConstants.CoralTarget.kTargetX;
    double path_y = target_pos.getY() - Constants.CoralSubsystemConstants.CoralTarget.kTargetY;

    //Multiply negative 1
    path_x *= 1;
    path_y *= 1;
    
     // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(path_x/2,path_y/2)), new Pose2d(path_x, path_y, new Rotation2d(0)),
        config);
/** 
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
*/

   /*  if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
        this.dontSeeTagTimer.reset();

        double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      */
      if(path_x < 2 && path_y < 2){ //was 10

        SmartDashboard.putNumber("Align/Camera/MoveX", path_x*100);
        SmartDashboard.putNumber("Align/Camera/MoveY", path_y*100);
        SmartDashboard.putNumber("Align/Camera/Rotrot", 0.0);

        SmartDashboard.putNumber("Align/Camera/getX", tX*100);
        SmartDashboard.putNumber("Align/Camera/getY", tY*100);

        double xSpeed = xController.calculate(path_x);
        double ySpeed = -yController.calculate(path_y);
        double rotValue = rotController.calculate(0.0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        //thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,       
        m_drivetrain::setModuleStates,
        m_drivetrain);
        //DriveSubsystem.ChassisSpeeds(xSpeed , ySpeed , rotValue);
        m_drivetrain.driveRobotRelative(new ChassisSpeeds(xSpeed , ySpeed , rotValue));

        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() || !xController.atSetpoint()) {
            stopTimer.reset();
        }
    }      
    else {
      m_drivetrain.driveRobotRelative(new ChassisSpeeds(0 , 0 , 0));
      //m_drivetrain.driveRobotRelative(stopspeeds);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return (this.dontSeeTagTimer.hasElapsed(3.0) ||
        this.stopTimer.hasElapsed(2.0)) || this.idleTimer.hasElapsed(1.5);
  }
}