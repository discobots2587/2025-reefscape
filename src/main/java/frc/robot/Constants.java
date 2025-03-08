// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second





    
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 30;
    public static final int kRearLeftDrivingCanId =40;
    public static final int kFrontRightDrivingCanId = 20;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 31;
    public static final int kRearLeftTurningCanId = 41;
    public static final int kFrontRightTurningCanId = 21;
    public static final int kRearRightTurningCanId = 11;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int k0pControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.2;
  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  public final class ElevatorSubsystemconstant {
    public static final double KDefualtMotorspeed = 0.5;
    public static final int ElevatorSubsystemCanId = 48;
  }
  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 4;
    public static final int kArmMotorCanId = 3;
    public static final int kFunnelMotorCanId = 9;
    public static final int FUNNEL_MOTOR_CURRENT_LIMIT = 10;
    public static final double FUNNEL_MOTOR_VOLTAGE_COMP = 12;
    public static final double FUNNEL_SPEED_DOWN = -0.1;
    public static final double FUNNEL_SPEED_UP = 0.1;
   

    public static final class ElevatorSetpoints {
      public static final int kIntake = 0;
      //we need to change with the funnel
      public static final int kFeederStation = 45;
      public static final int kLevel1 = 30;
      public static final int kLevel2 = 30;
      public static final int kLevel3 = 165;
      public static final int kLevel4 = 381;
    }

    public static final class ArmSetpoints {
      public static final double kIntake = 0.24;
      public static final double kFeederStation = 0.24;
      public static final double kLevel1 = 0.24;
      public static final double kLevel2 = 0.63;
      public static final double kLevel3 = 0.69;
      public static final double kLevel4 = 0.69;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
    }
  }
  public static final class AlgaeSubsystemConstants {
    public static final int kIntakeMotorCanId = 6;
    public static final int kPivotMotorCanId = 5;

    public static final class ArmSetpoints {
      public static final double kStow = 18.5;
      public static final double kHold = 11.5;
      public static final double kDown = 0;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.8;
      public static final double kReverse = -0.8;
      public static final double kHold = 0.25;
    }
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 45; // 45:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.035 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.64; // m
    public static final double kMaxElevatorHeightMeters = 1.65; // m

    public static final double kArmReduction = 50; // 50:1
    public static final double kArmLength = 0.46; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-75 ); // -90 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(85 ); // 85 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(50); // 80
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(150); // 180
    public static final double kIntakeShortBarLength = 0.3048;  // 0.1524
    public static final double kIntakeLongBarLength = 0.05;   // 0.3048
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }
  public final class AlgaeSubsystemConstant {
    public static final int kFunalMotorCanID = 26 ;
    public static final int kArmMotorCanID = 25;
  }
  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 25;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 10;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -0.1;
    public static final double CLIMBER_SPEED_UP = 0.1;
  }
}
