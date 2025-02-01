package frc.robot.Subsystems;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants.ElevatorPositionGroup;
import frc.robot.Util.Constants.constants_Elevator;
import frc.robot.Util.RobotMap.MAP_ELEVATOR;

public class Elevator extends SubsystemBase
{
    // Left = Right
    public SparkMax ELE_LEFT;
    public RelativeEncoder ELE_LEFT_ENCODER;
    public SparkClosedLoopController ELE_LEFT_PID;

    public SparkMax ELE_RIGHT;
    public RelativeEncoder ELE_RIGHT_ENCODER;
    public SparkClosedLoopController ELE_RIGHT_PID;

    // public MotionMagicVelocityVoltage motionMagicRequest;
    // public SparkMaxConfiguration elevatorConfig = new SparkMaxConfiguration();
    // public NeutralOut neutralOut;
    // public Slot0Configs ELE_CONFIG;
    // public FeedbackConfigs ELE_FEEDBACKCONFIG;
    // public ClosedLoopOutputType elevatorClosedLoopOutput = ClosedLoopOutputType.Voltage;

    public SparkMax ELE_ARM;
    public RelativeEncoder ELE_ARM_ENCODER;
    public SparkClosedLoopController ELE_ARM_PID;
    public SparkBaseConfig config;
    
    
    public boolean testBool = false;

            CANrange CANrange;
            CANrangeConfiguration configs;
    
    
    public Elevator()
    {
        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_ARM = new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_ARM_ENCODER = ELE_ARM.getEncoder();
        ELE_ARM_PID = ELE_ARM.getClosedLoopController();
        ELE_ARM.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_LEFT= new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_LEFT_ENCODER = ELE_LEFT.getEncoder();
        ELE_LEFT_PID = ELE_LEFT.getClosedLoopController();
        ELE_LEFT.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_RIGHT= new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_RIGHT_ENCODER = ELE_RIGHT.getEncoder();
        ELE_RIGHT_PID = ELE_RIGHT.getClosedLoopController();
        ELE_RIGHT.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

        CANrange = new CANrange(13);
        configs = new CANrangeConfiguration();
    }
    
    

    public double getPosition()
    {
        return ELE_LEFT_ENCODER.getPosition();
    }
    
    public boolean getGamePieceStored()
    {
        // CANrange CANrange;
        // CANrangeConfiguration configs;

        // CANrange = new CANrange(13);
        // configs = new CANrangeConfiguration();

        double coralDetectRange;
        CANrangeConfiguration configs = new CANrangeConfiguration();

       coralDetectRange = CANrange.getDistance().getValueAsDouble();
       if (coralDetectRange < 51) {
        return true; 
       } else {
        return false;
       }

    }
    
    public void setElevatorPosition(ElevatorPositionGroup position)
     {
        ELE_LEFT_PID.setReference(position.elevatorPosition.magnitude(), ControlType.kPosition);
        ELE_RIGHT_PID.setReference(position.elevatorPosition.magnitude(), ControlType.kPosition);
     }


     @Override
    public void periodic()
    {
        
    
    
    }

}
    
    
    
