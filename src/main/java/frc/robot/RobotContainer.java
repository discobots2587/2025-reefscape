// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AlgaeSubsystemConstant;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorSubsystemconstant;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AlgaeSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.AlgaeSubsystemConstants.IntakeSetpoints;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.funnelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

//<<<<<<< HEAD
import javax.security.auth.login.FailedLoginException;
//=======
//>>>>>>> 1211b516d9701d13e2756645fe8730233dfc2863

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.commands.scoreL3test;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
 // private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(ElevatorSubsystemconstant.ElevatorSubsystemCanId);
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();   
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final funnelSubsystem m_funnelSubsystem = new funnelSubsystem();
  
  //Controller Initialization
  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.k0pControllerPort);

  // The operator's controller
  private final JoystickButton m_liftX = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  private final JoystickButton m_liftY=  new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  private final JoystickButton m_liftB = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  private final JoystickButton m_liftA=  new JoystickButton(m_operatorController, XboxController.Button.kA.value);

  private final JoystickButton m_Level3DEAL = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_Level2DEAL = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);  

  // The driver's controller
  private final JoystickButton resetheading = new JoystickButton(m_driveController, XboxController.Button.kB.value);
  private final JoystickButton resetCoral = new JoystickButton(m_driveController, XboxController.Button.kStart.value);
  private final JoystickButton m_pivotIntake = new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_algaeScore = new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_moveFunnel = new JoystickButton(m_driveController, XboxController.Button.kX.value);  
  private final JoystickButton m_reverseFunnel = new JoystickButton(m_driveController, XboxController.Button.kY.value);

 private SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
    //
    

    // Register Named Commands
    NamedCommands.registerCommand("liftL3", m_coralSubSystem.setSetpointCommand(CoralSubsystem.Setpoint.kLevel3));
    NamedCommands.registerCommand("intakecoral", m_coralSubSystem.setSetpointCommand(CoralSubsystem.Setpoint.kFeederStation));
    NamedCommands.registerCommand("scoreCoral", m_coralSubSystem.scoreCoralCommand());
    NamedCommands.registerCommand("liftl4", m_coralSubSystem.setSetpointCommand(CoralSubsystem.Setpoint.kLevel4));


    NamedCommands.registerCommand("liftl0", new InstantCommand(() -> System.out.println("lift level 0")));
    NamedCommands.registerCommand("armscore", new InstantCommand(() -> System.out.println("lower arm to reef")));
    NamedCommands.registerCommand("reverse", new InstantCommand(() -> System.out.println("reverse from reef")));

    // Auto chooser
    
    autoChooser = AutoBuilder.buildAutoChooser();


    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Change this to match the name of your camera
    //PhotonCamera cameraL = new PhotonCamera("FrontL");
    //PhotonCamera cameraR = new PhotonCamera("FrontR");
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driveController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driveController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driveController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

// Set the ball intake to in/out when not running based on internal state
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand()); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /*new JoystickButton(m_driveController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));*/
    resetheading.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));

  //Motor Button X and y Control 
m_Level2DEAL
    .onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2DEAL));
m_Level3DEAL
    .onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3DEAL));
    


m_pivotIntake
    //.onTrue(new InstantCommand(() -> m_algaeSubsystem.setIntakePosition(Constants.AlgaeSubsystemConstants.IntakeSetpoints.kForward))); //NEEDS TUNING
    .onTrue(m_algaeSubsystem.runIntakeCommand());

    
    //algaesubsystem set point intke
/*m_pivotIntake.onTrue(m_algaeSubsystem.runIntakeCommand());
m_pivotOutake.onTrue(m_algaeSubsystem.reverseIntakeCommand());
*/
    


    // B Button -> Elevator/Arm to human player position, set ball intake to stow
    // when idle
     
    m_liftB
        .onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
               // .alongWith(m_algaeSubsystem.stowCommand()));  -- remove

    //GOAL: Merge setpoitns into 1 button, selectable level through a selector in smartdashboard
    //Get to a point where we can access all 4 levels with 
    // A Button -> Elevator/Arm to level 2 position
    m_liftA.onTrue(m_coralSubSystem.pickupCoralCommand());

    // X Button -> Elevator/Arm to level 3 position (BUTTON WE ARE USING THIS FOR SETPOINTS)
    m_liftX.onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_liftY.onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    resetCoral.onTrue (new InstantCommand(() -> m_coralSubSystem.resetCoral()));
    m_coralSubSystem.resetCoral();

    m_algaeScore.onTrue(m_algaeSubsystem.scoreAlgae());

    m_moveFunnel.whileTrue(new InstantCommand(() -> m_funnelSubsystem.runClimber(Constants.CoralSubsystemConstants.FUNNEL_SPEED_UP)));
    m_moveFunnel.whileFalse(new InstantCommand(() -> m_funnelSubsystem.runClimber(0)));
    m_reverseFunnel.whileTrue(new InstantCommand(() -> m_funnelSubsystem.runClimber(Constants.CoralSubsystemConstants.FUNNEL_SPEED_DOWN)));
    m_reverseFunnel.whileFalse(new InstantCommand(() -> m_funnelSubsystem.runClimber(0)));


 //Driver trigger controls
 // Right Trigger -> Run ball intake, set to leave out when idle
 CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
     m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
     m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());
    
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());
        //.whileTrue(new InstantCommand(() -> m_algaeSubsystem.setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kReverse)));

//Operator trigger controls

CommandXboxController m_operatorController = new CommandXboxController(OIConstants.k0pControllerPort);
    
    /**
     * POV is a direction on the D-Pad or directional arrow pad of the controller,
     * the direction of this will be different depending on how your winch is wound
     */

    //Driver controller DPAD
    m_driverController.pov(0).whileTrue(new InstantCommand(() ->m_climberSubsystem.runClimber(Constants.ClimberConstants.CLIMBER_SPEED_UP)));
    m_driverController.pov(0).onFalse(new InstantCommand(() ->m_climberSubsystem.runClimber(0.0)));
    m_driverController.pov(180).whileTrue(new InstantCommand(() ->m_climberSubsystem.runClimber(Constants.ClimberConstants.CLIMBER_SPEED_DOWN)));
    m_driverController.pov(180).onFalse(new InstantCommand(() ->m_climberSubsystem.runClimber(0.0)));

    //Operator Controller DPAD
     m_operatorController.pov(90).onTrue(m_coralSubSystem.scoreCoralCommand());
     m_operatorController.pov(0).onTrue(m_coralSubSystem.setSetpointCommand(CoralSubsystem.Setpoint.kFeederStation));
     m_operatorController.pov(180).onTrue(m_coralSubSystem.setSetpointCommand(CoralSubsystem.Setpoint.kIntake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*   REMOVE  THIS COMMENT TO ENABLE TRAJECTORY AUTO (also remove lots of unused import statements)
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
    return autoChooser.getSelected();  // use this line for Path Planner Selector
  }
}
