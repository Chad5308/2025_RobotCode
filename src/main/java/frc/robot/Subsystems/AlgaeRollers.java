package frc.robot.Subsystems;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public SparkMax PITCH;
    public RelativeEncoder PITCH_ENCODER;
    public SparkClosedLoopController PITCH_PID;
    public SparkBaseConfig configPitch;

    public CANrange CANrange;
    public CANrangeConfiguration sensorConfigs;

    public boolean testBool = false;

    public AlgaeRollers()
    {
        configRollers = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ROLLERS = new SparkMax(MAP_ALGAE.ALGAE_ROLLERS, MotorType.kBrushless);
        ROLLERS_ENCODER = ROLLERS.getEncoder();
        ROLLERS_PID = ROLLERS.getClosedLoopController();
        ROLLERS.configure(configRollers, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configPitch = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        configPitch.encoder.positionConversionFactor(constants_Rollers.ROLLER_GEAR_RATIO);
        PITCH = new SparkMax(MAP_ALGAE.ALGAE_PITCH, MotorType.kBrushless);
        PITCH_ENCODER = PITCH.getEncoder();
        PITCH_PID = PITCH.getClosedLoopController();
        PITCH.configure(configPitch, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CANrange = new CANrange(MAP_ALGAE.ALGAE_SENSOR);
        sensorConfigs = new CANrangeConfiguration();
        CANrange.getConfigurator().apply(sensorConfigs);
    }



    public double getPosition()
    {
        return PITCH_ENCODER.getPosition() * 360;
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
       return getDistance() < 10;
    }

    public void setAlgaeIntake(AlgaePositionGroup group) //degrees & m/s
    {
        PITCH_PID.setReference(group.intakeAngle, ControlType.kPosition);
        ROLLERS_PID.setReference(group.rollersRPM, ControlType.kVelocity);
    }

    public boolean isRollersInPosition(AlgaePositionGroup position)
    {
        return (Math.abs(getPosition() - position.intakeAngle) < constants_Rollers.ROLLER_ANGLE_TOLERANCE) && (Math.abs(getSpeed() - position.rollersRPM) < constants_Rollers.ROLLER_RPM_TOLERANCE);
    }

    

    public Command homePosition()
    {
        return Commands.runOnce(()->
        {
            //TODO Write code here to set home position of the arm
        });
    }

    //TODO Follow the same premice as shown above and create a method returning a Command that sets the arm to an intake state



    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Algae Detection", getGamePieceCollected());
    }
}
