package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralSubsystemConstants;;

public class funnelSubsystem extends SubsystemBase {

    private final SparkMax funnelMotor;

    /**
     * This subsytem control the climber.
     */
    public funnelSubsystem () {

    // Set up the climb motor as a brushless motor
    funnelMotor = new SparkMax(CoralSubsystemConstants.kFunnelMotorCanId, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    funnelMotor.setCANTimeout(250);

    // Create and apply configuration for climb motor. Voltage compensation helps
    // the climb behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the climb stalls.
    SparkMaxConfig funnelConfig = new SparkMaxConfig();
    funnelConfig.voltageCompensation(CoralSubsystemConstants.FUNNEL_MOTOR_VOLTAGE_COMP);
    funnelConfig.smartCurrentLimit(CoralSubsystemConstants.FUNNEL_MOTOR_CURRENT_LIMIT);
    funnelConfig.idleMode(IdleMode.kBrake);
    funnelMotor.configure(funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        funnelMotor.set(speed);
    }

}
