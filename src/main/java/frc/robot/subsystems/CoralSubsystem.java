package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;

import org.photonvision.PhotonCamera;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.LEDModes;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.ElevatorSubsystemconstant;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import static java.util.stream.Collectors.toUnmodifiableSet;
import java.util.Set;
import java.util.stream.Stream;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.LEDSubsystem;


public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kIntake,
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel2DEAL,
    kLevel3,
    kLevel3DEAL,
    kLevel4;
  }

  public Setpoint lastSetpoint = Setpoint.kIntake; // keep track of level to help with scoring arm down
    // ID of the tags on the reef
  private static final Set<Integer> FIDUCIAL_IDS = Stream.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11)
      .collect(toUnmodifiableSet());

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor =
      new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();
  private AbsoluteEncoder angleEncoder = armMotor.getAbsoluteEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor =
      new SparkMax(Constants.CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless); //Maybe use the constant???? TODO
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax intakeMotor =
      new SparkMax(59, MotorType.kBrushless); //What is happening here? TODO

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kIntake;
  private double elevatorCurrentTarget = ElevatorSetpoints.kIntake;
  private double angleCurrentTarget = ArmSetpoints.kIntake;
  private double initialAngle = 1;


  // Simulation setup and variables
  private DCMotor elevatorMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);

  private DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
          SimulationRobotConstants.kArmLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));
  private final MechanismLigament2d m_armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));
  public final  PhotonCamera cameraL = new PhotonCamera("FrontL");
  public final  PhotonCamera cameraR = new PhotonCamera("FrontR");
    // PWM port 9
  // Must be a PWM header, not MXP or DIO

  public final LEDSubsystem m_led = new LEDSubsystem(); 



  public CoralSubsystem() {
    //initialAngle = armEncoder.getPosition();
    initialAngle = angleEncoder.getPosition();
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    armMotor.configure(
        Configs.CoralSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotor.configure(
        Configs.CoralSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.configure(
        Configs.CoralSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    

    // Display mechanism2d
    SmartDashboard.putData("Coral Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);

    
    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
    armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      //HACK armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              armCurrentTarget = ArmSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kIntake:
              armCurrentTarget = ArmSetpoints.kIntake;
              elevatorCurrentTarget = ElevatorSetpoints.kIntake;
              break;
            case kLevel1:
              armCurrentTarget = ArmSetpoints.kLevel1;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              armCurrentTarget = ArmSetpoints.kLevel2;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              armCurrentTarget = ArmSetpoints.kLevel3;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              armCurrentTarget = ArmSetpoints.kLevel4;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
            case kLevel2DEAL:
              armCurrentTarget = ArmSetpoints.kArmDEAL;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2DEAL;
              break;
            case kLevel3DEAL:
              armCurrentTarget = ArmSetpoints.kArmDEAL;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3DEAL;
          }
          lastSetpoint = setpoint; // retain the last setpoint for scoring arm down
        });
  }
  /* Scores the coral by moving arm down
   * This is a separate command because the arm needs to move down after the elevator has reached
   */
  public Command scoreCoralCommand(){
    return this.runOnce(
      () -> {
        double arm_delta = .22;
        double arm_delta4 = 0.3;
        switch (this.lastSetpoint) {
          case kLevel1:
            if (armCurrentTarget == ArmSetpoints.kLevel1) {
              armCurrentTarget = ArmSetpoints.kLevel1 - arm_delta;
            }
            break;
          case kLevel2:
            if (armCurrentTarget == ArmSetpoints.kLevel2) {
              armCurrentTarget = ArmSetpoints.kLevel2 - arm_delta;
            }
            break;
          case kLevel3:
            if (armCurrentTarget == ArmSetpoints.kLevel3) {
               armCurrentTarget = ArmSetpoints.kLevel3 - arm_delta;
            }
            break;
          case kLevel4:
            if (armCurrentTarget == ArmSetpoints.kLevel4) {
              armCurrentTarget = ArmSetpoints.kLevel4 - arm_delta4; 
            }
            break;
        }
      });
  }

  //This Command will allow the operator to pickup coral with one button
  //UNTESTED
  public Command pickupCoralCommand(){
    return this.runOnce(
      () -> {
        Double waitDouble = 2.0;
        setSetpointCommand(CoralSubsystem.Setpoint.kIntake);
        Commands.waitSeconds(waitDouble);
        setSetpointCommand(CoralSubsystem.Setpoint.kFeederStation);
      }
    );
  }

    /**
   * Sets the speed of the single motor.
   *
   * @param speed The speed to set the motor, between -1.0 and 1.0.
   */
  public void setArmSpeed(double speed) {
    // Ensure the speed is within the valid range [-1.0, 1.0]
   // m_armSpeed = speed;
    armMotor.set(speed);
  }
    /**
   * Sets the speed of the single motor.
   *
   * @param speed The speed to set the motor, between -1.0 and 1.0.
   */
  public void setLiftSpeed(double speed) {
    // Ensure the speed is within the valid range [-1.0, 1.0]
   // m_armSpeed = speed;
    elevatorMotor.set(speed);
  }
  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
  }

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
  }
  public void resetCoral (){
    elevatorEncoder.setPosition(0);
      // HACK armEncoder.setPosition(0);
  }
  public void initializeCoral(){
    armCurrentTarget = 0;
    elevatorCurrentTarget = 0;
  }
  @Override

  public void periodic() {
    if(Configs.CoralSubsystem.setpointMode) {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();
    }
    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Coral/ArmAngle/Target Angle", angleCurrentTarget);
    SmartDashboard.putNumber("Coral/ArmAngle/Actual Angle", angleEncoder.getPosition());
    SmartDashboard.putNumber("Coral/ArmAngle/Initial Angle", initialAngle);
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
    if ( elevatorMotor.getReverseLimitSwitch().isPressed()) {
      SmartDashboard.putNumber("Coral/Elevator/Down", 1);
    } else {
      SmartDashboard.putNumber("Coral/Elevator/Down", 0);
    }
    // return camera positions
    Transform3d cameraLeft = updateCameraPositions(cameraL);
    SmartDashboard.putNumber("Coral/cameraL/getY", cameraLeft.getY());
    SmartDashboard.putNumber("Coral/cameraL/getX", cameraLeft.getX());
  
    
    Transform3d cameraRight = updateCameraPositions(cameraR);

    
    SmartDashboard.putNumber("Coral/cameraR/getY", cameraRight.getY());
    SmartDashboard.putNumber("Coral/cameraR/getX", cameraRight.getX());
    boolean scoreR = false , scoreL = false;
    if (cameraLeft.getY() > -.03 && cameraLeft.getY() < .03) {
      scoreR = true;
    }
    if (cameraRight.getY() < .03 && cameraRight.getY() > -.03) {
      scoreL = true;
    }
    if (scoreR) {
      m_led.setStatus(LEDModes.kAlignR);
    } else if (scoreL) {
      m_led.setStatus(LEDModes.kAlignL);
    } else if (elevatorEncoder.getPosition()< ArmSetpoints. kFeederStation){
      m_led.setStatus(LEDModes.kFeeder);
    } else {
      m_led.setStatus(LEDModes.kNone);
    }

    SmartDashboard.putBoolean ("Coral/cameraR/scoreR", scoreR);
    SmartDashboard.putBoolean ("Coral/cameraL/scoreL", scoreL);

    // todo add test for getX values for scoring on left or right branch
    
    // todo add camera to target threshold values for scoring at different levels

  /* 
    // Update mechanism2d
    m_elevatorMech2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    armEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );*/
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);
    // SimBattery is updated in Robot.java
  }
  public Transform3d updateCameraPositions(PhotonCamera photonCamera) {
    Transform3d  cameraToTarget = new Transform3d();
    var photoResults = photonCamera.getAllUnreadResults();
    var lastTagResult = photoResults.stream()
        .filter(result -> result.hasTargets())
        .flatMap(result -> result.getTargets().stream())
        .filter(target -> FIDUCIAL_IDS.contains(target.getFiducialId()))
        .findFirst();

    if (lastTagResult.isPresent()) {
      PhotonTrackedTarget tag = lastTagResult.get();
      cameraToTarget = tag.bestCameraToTarget;
     //Set Z cameraPose = cameraPose.set;
      double yValue = cameraToTarget.getY();
      double xValue = cameraToTarget.getX();
      SmartDashboard.putNumber("Coral/camera/getY", yValue);
      SmartDashboard.putNumber("Coral/camera/getX", xValue);
      SmartDashboard.putNumber("Coral/camera/tag", tag.getFiducialId());
    }
    return cameraToTarget;
  }
}
