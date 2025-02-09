// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.security.Policy;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {


  // Define the Spark Max motor controller
  private final SparkMax m_singleMotor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_pidController;
  private DigitalInput homeSwitch;

  //target angle 
  private double m_targetAngle;

  //flag to indicate if closed-loop (PID) is active or not;
  private boolean m_closedLoopEnable = false;
   
  //store target rotations
  private double m_setpointRotations;

  // Speed variable for debugging and SmartDashboard control
  private double m_speed;
  private double m_armSpeed = 0;


  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

    // set motor angle 
    public static final double MIN_ANGLE = -45;  // degrees
    public static final double MAX_ANGLE = +60;  // degrees

  
  /**
   * Creates a new SingleMotorSubsystem.
   *
   * @param motorCANID The CAN ID of the motor controller.
   * @param homeSwitchPort The DIO port for the home limit switch
   */
  public ElevatorSubsystem(int motorCANID) {
    // Initialize the motor controller with the specified CAN ID
    m_singleMotor = new SparkMax(motorCANID, MotorType.kBrushless); // Assume it's a brushless motor

    m_encoder = m_singleMotor.getEncoder();
    m_pidController = new SparkMaxPIDController(kP, kI, kD);
    
    
  
    // Set default speed to 0
    m_speed = -1;
    m_targetAngle = getMotorAngle();
    // Add a SmartDashboard entry for motor speed adjustment
    SmartDashboard.putNumber("Single Motor Speed", m_speed);

   

    m_targetAngle = getMotorAngle();
    SmartDashboard.putNumber("Single Motor Speed", m_speed);
    SmartDashboard.putNumber("single Motor Manual Speed",0);
    SmartDashboard.putNumber("Elevator Target Angle", m_targetAngle);
    SmartDashboard.putBoolean("Closed Loop Enable", m_closedLoopEnable);
    }
    
    public Void setTargetRotations (double rotations) {
      m_setpointRotations = rotations;
      SmartDashboard.putNumber("Elevator Target Rotations", m_setpointRotations);
            return null;
    } 
  @Override
  public void periodic() {
    // read the speed value from the smartdashboard
    m_speed = SmartDashboard.getNumber("Single Motor Speed", 0);
    SmartDashboard.putNumber("Single Motor Actual Speed", m_armSpeed);
    SmartDashboard.putNumber("Arm Angle", getMotorAngle());
  
      if (m_closedLoopEnable) {
        updatePID();
      }
    SmartDashboard.putBoolean("Elevator Homed",  getHomeSwitch());
  }

    public void updatePID() {
      double clampedTarget = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, m_setpointRotations * 360.0)); // Convert rotations to degrees
      double pidOutput = m_pidController.calculate(getMotorAngle(), clampedTarget);
      m_singleMotor.set(pidOutput);
    }
    /**
     * Moves motor to specific rotation position
     * @param rotations Desired motor rotations.
     */
    public void moveToRotation(double elevatorRotations){
     double motorRotations = elevatorRotations *50;
     m_pidController.setReference(motorRotations, SparkMax.ControlType.kPosition);
    }
    
    public void setTargetAngle(double targetAngle) {
      m_targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
      SmartDashboard.putNumber("Elevator Target Angle", m_targetAngle);

    }
public void setMotorAngle(double angle) {
    double clampedAngle =  Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));

    double elevatorRotations = clampedAngle / 360.0;
    double motorRotations = elevatorRotations * 50.0;

    // compute motor output using pid
    double output = m_pidController.calculate(getMotorAngle(), angle);
    m_singleMotor.set(output);
  }
  /*
   * Reset Encode to zero
   */
  public void resetEncoder(){
    m_encoder.setPosition(0);
    SmartDashboard.putNumber("Encoder Reset", 0);
  }


public void holdPosition(){
  m_targetAngle = getMotorAngle();
  setClosedLoopEnable(true);
    SmartDashboard.putString("Elevator Status", "Holding Position");
  }
  
     private void setClosedLoopEnable(boolean enable) {
      m_closedLoopEnable = enable;
      SmartDashboard.putBoolean("Closed Loop Enable", enable);
    }
  /**
   * Returns true if the home switch is pressed
   * @return True if the elevator is at home position 
   */
  public boolean getHomeSwitch(){
    return !homeSwitch.get();
  }
  
  
    /**
   * Sets the speed of the single motor.
   *
   * @param speed The speed to set the motor, between -1.0 and 1.0.
   */
  public void setMotorSpeed(double speed) {
    // Ensure the speed is within the valid range [-1.0, 1.0]
    m_armSpeed = speed;
    m_singleMotor.set(speed);
  }
  
  /**
 * Returns the current elevator angle in degrees.
 * Adjust the conversion factor based on your encoder’s units and gearing.
 *
 * @return The current angle in degrees.
 */
public double getMotorAngle() {
  // Example conversion: assume the encoder returns rotations
  return m_encoder.getPosition() * 360.0;
}


  /*
   * moves the motor to the specified angle using the PID control
   * 
   * @param angle the desired angle in degreen 
   */
  public void setmotorgle(double angle) {
    // clamp the angle to the allowed range
    angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));


     // Compute motor output using PID
     double output = m_pidController.calculate(getMotorAngle(), angle);

      // Send output to motor
      m_singleMotor.set(output);
  }
    
    /**
     * Stops the motor by setting its speed to 0.
     */
    public void stopMotor() {
      m_singleMotor.set(0);
    }


  /**
   * Returns the current speed of the motor.
   *
   * @return The current motor speed.
   */
  public double getMotorSpeed() {
    return m_speed;
  }


}


