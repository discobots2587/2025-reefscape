// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.CoralSubsystemConstants.CoralTarget;
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
    //private double tagID;
    

  public AlignToBranch(DriveSubsystem m_drivetrain, CoralSubsystem coral, boolean is_right) {
    double p_x = 20,p_y = 20, p_r = 2;
    xController = new PIDController(p_x, 0.0, 0);  // Vertical movement
    yController = new PIDController(p_y, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(p_r, 0, 0);  // Rotation
    this.m_drivetrain = m_drivetrain;
    this.m_coral = coral;
    //this.tagID = tag;
    this.isRightScore = is_right;
    addRequirements(m_drivetrain);
  }

  public int count = 0;
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    count = 0;
    /* 
    this.idleTimer = new Timer();
    this.idleTimer.start();
*/

    
    xController.setSetpoint(Constants.CoralSubsystemConstants.CoralTarget.kTargetX);
    xController.setTolerance(Constants.CoralSubsystemConstants.CoralTarget.kTargetXTol);

    yController.setSetpoint(Constants.CoralSubsystemConstants.CoralTarget.kTargetY);
    yController.setTolerance(Constants.CoralSubsystemConstants.CoralTarget.kTargetYTol);

    //tagID = -1;
    //results = camera.getAllUnreadResults();
    //tagID = target.getFidcialID();
  }

  @Override
  public void execute() { 
    Transform3d target_pos = m_coral.getTargetPos(isRightScore);
    double path_x = target_pos.getX(); 
    double path_y = target_pos.getY();


 /*
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
*/
      if(path_x < 2 && path_y < 2){ //was 10
        count ++;

        dontSeeTagTimer.reset();

        double xSpeed = xController.calculate(path_x,Constants.CoralSubsystemConstants.CoralTarget.kTargetX);
        double ySpeed = -yController.calculate(path_y,Constants.CoralSubsystemConstants.CoralTarget.kTargetY);
        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);

        SmartDashboard.putNumber("ALIGN/MoveX", xSpeed);
        SmartDashboard.putNumber("ALIGN/MoveY", ySpeed);
    
        System.out.println("XSPEED: " + xSpeed);
        System.out.println("YSPEED" + ySpeed);

        SmartDashboard.putNumber("Align/Camera/MoveX", path_x*100);
        SmartDashboard.putNumber("Align/Camera/MoveY", path_y*100);
        SmartDashboard.putNumber("Align/Camera/Rotrot", 0.0);
        SmartDashboard.putNumber("Align/COUNT", count);

        // SmartDashboard.putNumber("Align/Camera/getX", tX*100);
        // SmartDashboard.putNumber("Align/Camera/getY", tY*100);

     //   double xSpeed = xController.calculate(path_x);
       // double ySpeed = -yController.calculate(path_y);

        
        SmartDashboard.putNumber("Align/Camera/xSpeed",xSpeed);
        SmartDashboard.putNumber("Align/Camera/ySpeed", ySpeed);

        double rotValue = rotController.calculate(0.0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
        m_drivetrain.drive(xSpeed, ySpeed, rotValue, false);

        if (!yController.atSetpoint()||!xController.atSetpoint()) {
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
  public boolean isFinished() {
    //return this.dontSeeTagTimer.hasElapsed(0.5) || stopTimer.hasElapsed(1);

    Transform3d target_pos = m_coral.getTargetPos(isRightScore); 
    double tX = target_pos.getX();
    double tY = target_pos.getY();

    boolean close = false;
    if 
    (
    (Math.abs(tX - CoralTarget.kTargetX) < CoralTarget.kTargetXTol) || 
    (Math.abs(tY - CoralTarget.kTargetY) < CoralTarget.kTargetYTol)
    )
    {
      close = true;
     // end(true);
    }
    SmartDashboard.putBoolean("AlignToBranch/Close", close);
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    //boolean done = (this.dontSeeTagTimer.hasElapsed(2.0) || close ||        
      //  this.stopTimer.hasElapsed(3.0));
    if(!close && tX <2){
      System.out.println("Align Command Done, CLOSE = " + close +" "+ tX + " " + tY);
    }
        return close;
  }
}