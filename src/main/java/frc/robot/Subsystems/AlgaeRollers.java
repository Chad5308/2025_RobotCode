package frc.robot.Subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    }



    public double getPosition()
    {
        return PITCH_ENCODER.getPosition() * 360;
    }

    public double getSpeed()
    {
        return ROLLERS_ENCODER.getVelocity();
    }
        
    public boolean getGamePieceStored()
    {
        return false;
        //TODO Code this distance/proxy sensor in
    }

    public boolean getGamePieceCollected()
    {
        return false;
        //TODO Code this distance/proxy sensor in
    }


    public void setPosition(double position) //degrees
    {
        PITCH_PID.setReference(position, ControlType.kPosition);
    }

    public void setSpeed(double speed) //rps
    {
        ROLLERS_PID.setReference(speed, ControlType.kVelocity);
    }
}
