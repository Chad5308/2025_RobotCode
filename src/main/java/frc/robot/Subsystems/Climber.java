package frc.robot.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants.ClimberPositionGroup;
import frc.robot.Util.Constants.constants_Climber;
import frc.robot.Util.RobotMap.MAP_CLIMBER;

public class Climber extends SubsystemBase 
{
    public SparkMax CLIMBER_SPARKMAX;
    public RelativeEncoder CLIMBER_ENCODER;
    public SparkClosedLoopController CLIMBER_PID;
    public SparkBaseConfig CLIMBER_CONFIG;
    // public UsbCamera climber_Camera;
    //Create all instance variables
    //1 Neo motor, including its encoder, pid controller, and configuration
    

    public Climber()
    {
        //Instantiate neo stuff
        CLIMBER_CONFIG = new SparkMaxConfig();
        CLIMBER_CONFIG.encoder.positionConversionFactor(constants_Climber.CLIMBER_GEAR_RATIO);
        CLIMBER_CONFIG.inverted(constants_Climber.CLIMBER_INVERTED);
        CLIMBER_CONFIG.idleMode(IdleMode.kBrake);

        CLIMBER_SPARKMAX = new SparkMax(MAP_CLIMBER.CLIMB_SPARKMAX, MotorType.kBrushless);
        CLIMBER_ENCODER =  CLIMBER_SPARKMAX.getEncoder();
        CLIMBER_PID = CLIMBER_SPARKMAX.getClosedLoopController();
        CLIMBER_SPARKMAX.configure(CLIMBER_CONFIG, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
        CLIMBER_ENCODER.setPosition(0);
        // climber_Camera = CameraServer.startAutomaticCapture(1);
        
    }

    public double getPosition()
    {
        return CLIMBER_ENCODER.getPosition();
    }

    public void zeroPosition()
    {
        CLIMBER_ENCODER.setPosition(0);
    }
    
     public Command climberDown()
    {
        return Commands.runOnce(()->
        {
            CLIMBER_SPARKMAX.set(-1);
        });
    }

     public Command climberUp()
    {
        return Commands.runOnce(()->
        {
            CLIMBER_SPARKMAX.set(1);
        });
    }

     public Command stopClimber()
    {
        return Commands.runOnce(()->
        {
            CLIMBER_SPARKMAX.set(0);
        });
    }

    public void setClimber(ClimberPositionGroup group)
    {
        while(getPosition()>group.angle)
        {
            CLIMBER_SPARKMAX.set(-1);
        }
        CLIMBER_SPARKMAX.set(0);
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Climber position", getPosition());
    } 


    
}
