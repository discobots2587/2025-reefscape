package frc.robot.subsystems;

import java.security.PublicKey;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climbMotor;

    /**
     * This subsytem control the climber.
     */
    public ClimberSubsystem () {

    // Set up the climb motor as a brushless motor
    climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    climbMotor.setCANTimeout(250);

    // Create and apply configuration for climb motor. Voltage compensation helps
    // the climb behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the climb stalls.
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
    climbConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
       // SmartDashboard.putNumber("Timer", ClimberSubsystem.time);
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        climbMotor.set(speed);
    }

    
    public Command autoClimberCommand(){
        return this.runOnce(() -> {
            final Timer m_time = new Timer();
            m_time.restart();
            //new SequentialCommandGroup(
            //new InstantCommand(()-> runClimber(Constants.ClimberConstants.CLIMBER_SPEED_DOWN))   );
            
            //new WaitCommand(2),
            //new InstantCommand(()-> runClimber(0)));
            while (!m_time.hasElapsed(2.0)){
                runClimber(Constants.ClimberConstants.CLIMBER_SPEED_DOWN);
                /**
                if (Timer.getMatchTime() > 3){
                    autoStopClimber();
                }
                    */
            }; 
            runClimber(0);
        
        });
    }

    
    public Command autoStopClimber(){
        return this.run(() -> {
            runClimber(0);
        }
         );
    }
        




}