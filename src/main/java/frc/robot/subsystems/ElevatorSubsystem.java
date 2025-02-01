// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  // Define the Spark Max motor controller
  private final SparkMax m_singleMotor;

  // Speed variable for debugging and SmartDashboard control
  private double m_speed;

  private double m_armSpeed = 0;

  /**
   * Creates a new SingleMotorSubsystem.
   *
   * @param motorCANID The CAN ID of the motor controller.
   */
  public ElevatorSubsystem(int motorCANID) {
    // Initialize the motor controller with the specified CAN ID
    m_singleMotor = new SparkMax(motorCANID, MotorType.kBrushed); // Assume it's a brushless motor

    
    // Set default speed to 0
    m_speed = -1;

    // Add a SmartDashboard entry for motor speed adjustment
    SmartDashboard.putNumber("Single Motor Speed", m_speed);
  }

  @Override
  public void periodic() {
    // Read the speed value from the SmartDashboard for dynamic control
    m_speed = SmartDashboard.getNumber("Single Motor Speed", 0);

    // Set the motor speed
    //setMotorSpeed(m_speed);

    // Display the current motor speed on the SmartDashboard
    SmartDashboard.putNumber("Single Motor Actual Speed", m_armSpeed);
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

