package frc.robot.Subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants.AlgaePositionGroup;
import frc.robot.Util.Constants.constants_Rollers;
import frc.robot.Util.RobotMap.MAP_ALGAE;

public class AlgaeRollers extends SubsystemBase
{

    public SparkMax ROLLERS;
    public RelativeEncoder ROLLERS_ENCODER;
    public SparkClosedLoopController ROLLERS_PID;
    public SparkBaseConfig configRollers;

    public CANrange CANrange;
    public CANrangeConfiguration sensorConfigs;

    public boolean ALGAE_OVERRIDE = false;

    public DutyCycleEncoder absoluteEncoder;

    //TODO Zeroing the Algae Rollers and converting to degrees
    /*
     * 1. when the robot is off move the Intake all the way up
     * 2. Push code to the robot so that the intake is reset to 0 and temporarily secure the intake in its home positon
     * 4. Make sure both the Absolute encoder and Relative encoder are increasing positivly when moving down, if not, adjust the one that isnt moving the right way by flipping their coresponding "Inverted" boolean in the Constants_Rollers file, you can also check to make sure the intake rollers increase positivily when rotating inward
     * 6. Note down the rotations of the Absolute encoder position when in the home position in the variable "ABS_OFFSET" in the Constants_Rollers file
     * 7. Push the code to the robot and ur done!!
     */

    public AlgaeRollers()
    {
        absoluteEncoder = new DutyCycleEncoder(MAP_ALGAE.ALGAE_ABS_ENCODER_PWM_PORT);
        absoluteEncoder.setInverted(constants_Rollers.ABS_INVERTED);

        configRollers = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(constants_Rollers.ROLLER_P, constants_Rollers.ROLLER_I, constants_Rollers.ROLLER_D, constants_Rollers.ROLLER_FF, ClosedLoopSlot.kSlot0));
        configRollers.encoder.positionConversionFactor(1);
        configRollers.idleMode(IdleMode.kBrake);
        ROLLERS = new SparkMax(MAP_ALGAE.ALGAE_ROLLERS, MotorType.kBrushless);
        ROLLERS_ENCODER = ROLLERS.getEncoder();
        ROLLERS_PID = ROLLERS.getClosedLoopController();
        ROLLERS.configure(configRollers.inverted(constants_Rollers.ROLLER_INVERTED), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CANrange = new CANrange(MAP_ALGAE.ALGAE_CANRANGE);
        sensorConfigs = new CANrangeConfiguration();
        CANrange.getConfigurator().apply(sensorConfigs);

    }



    public double getPosition()
    {
        return (absoluteEncoder.get() * constants_Rollers.ANGLE_TO_DEGREES) - constants_Rollers.ABS_OFFSET;
    }

    public double getSpeed()
    {
        return ROLLERS_ENCODER.getVelocity();
    }

    public double getDistance()
    {
        return CANrange.getDistance().getValueAsDouble()* 39.3701; //Meters to Inches
    }
        
    public boolean getGamePieceCollected()
    {      
    //    return (getDistance() < constants_Rollers.CANRANGE_ROLLERS_DISTANCE_LIMIT) || ALGAE_OVERRIDE;
        return ALGAE_OVERRIDE;
    }

    public void intakeAlgae()
    {
        ROLLERS.set(0.50);
    }

    public void retractIntakeNone()
    {
        while(!isRollersInPosition(90))
         {
             ROLLERS.set(-0.25);
         }
         ROLLERS.set(0);
          ROLLERS.set(-0.50);
          try {
             Thread.sleep(500);
         } catch (InterruptedException e) {
             // TODO Auto-generated catch block
            // e.printStackTrace();
         }
          ROLLERS.set(0);
    }
    

    // public void retractIntakeAlgae()
    // {
    //     while(!isRollersInPosition(60))
    //     {
    //         ROLLERS.set(-0.25);
    //     }
    // }

    public boolean isRollersInPosition(double position)
    {
        // return (Math.abs(getPosition()- position) < constants_Rollers.ROLLER_ANGLE_TOLERANCE);
        return true;
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Algae Detection", getGamePieceCollected());
        SmartDashboard.putNumber("Algae Absolute Angle Encoder", getPosition());
        SmartDashboard.putNumber("Algae Relative Roller Encoder", ROLLERS_ENCODER.getPosition());
    }
}
