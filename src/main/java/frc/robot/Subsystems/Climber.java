package frc.robot.Subsystems;
import edu.wpi.first.units.Units;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import 

edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RobotMap;
import frc.robot.Util.Constants.AlgaePositionGroup;
import frc.robot.Util.Constants.ClimberPositionGroup;
import frc.robot.Util.Constants.constants_Climber;
import frc.robot.Util.RobotMap.MAP_CLIMBER;

public class Climber extends SubsystemBase 
{
    public SparkMax CLIMBER_SPARKMAX;
    public RelativeEncoder CLIMBER_ENCODER;
    public SparkClosedLoopController CLIMBER_PID;
    public SparkBaseConfig CLIMBER_CONFIG;
    //Create all instance variables
    //1 Neo motor, including its encoder, pid controller, and configuration
    

    public Climber()
    {
        //Instantiate neo stuff
        CLIMBER_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(constants_Climber.CLIMBER_P, constants_Climber.CLIMBER_I, constants_Climber.CLIMBER_D, constants_Climber.CLIMBER_FF, ClosedLoopSlot.kSlot0));
        CLIMBER_CONFIG.encoder.positionConversionFactor(constants_Climber.CLIMBER_GEAR_RATIO);
        CLIMBER_CONFIG.inverted(constants_Climber.CLIMBER_INVERTED);
        CLIMBER_CONFIG.idleMode(IdleMode.kBrake);

        CLIMBER_SPARKMAX = new SparkMax(MAP_CLIMBER.CLIMB_SPARKMAX, MotorType.kBrushless);
        CLIMBER_ENCODER =  CLIMBER_SPARKMAX.getEncoder();
        CLIMBER_PID = CLIMBER_SPARKMAX.getClosedLoopController();
        CLIMBER_SPARKMAX.configure(CLIMBER_CONFIG, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //convert the neos rotation to percentage once we figure out how far the motor has to travel to fully climb
            
        }
    



    public double getPosition()
    {
        return CLIMBER_SPARKMAX.get();
        
    }
    
    public void setPosition(double angle)
    {
        CLIMBER_SPARKMAX.set(angle);
    }

    public void setClimber(ClimberPositionGroup group)
    {
        CLIMBER_PID.setReference(group.angle, ControlType.kPosition);
    }


    
}
