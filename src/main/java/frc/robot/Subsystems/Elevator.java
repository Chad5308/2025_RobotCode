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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorPositionGroup;
import frc.robot.Util.Constants.constants_Elevator;
import frc.robot.Util.RobotMap.MAP_ELEVATOR;

public class Elevator extends SubsystemBase
{
    public SparkMax ELE_LEFT;
    public RelativeEncoder ELE_LEFT_ENCODER;
    public SparkClosedLoopController ELE_LEFT_PID;
    public SparkBaseConfig LEFT_CONFIG;

    public SparkMax ELE_RIGHT;
    public RelativeEncoder ELE_RIGHT_ENCODER;
    public SparkClosedLoopController ELE_RIGHT_PID;
    public SparkBaseConfig RIGHT_CONFIG;

    
    public SparkMax ELE_ROLLER;
    public RelativeEncoder ELE_ROLLER_ENCODER;
    public SparkClosedLoopController ELE_ROLLER_PID;
    public SparkBaseConfig ROLLER_CONFIG;
    
    public boolean testBool = false;

    public CANrange CANrange;
    public CANrangeConfiguration sensorConfigs;
    
    
    public Elevator()
    {
        sensorConfigs = new CANrangeConfiguration();
        CANrange = new CANrange(MAP_ELEVATOR.ELEVATOR_SENSOR);
        CANrange.getConfigurator().apply(sensorConfigs);

        LEFT_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.000075, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0));
        LEFT_CONFIG.encoder.positionConversionFactor(Constants.constants_Elevator.ELEVATOR_GEAR_RATIO);
        LEFT_CONFIG.inverted(constants_Elevator.LEFT_INVERTED);
        LEFT_CONFIG.idleMode(IdleMode.kBrake);

        RIGHT_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.000075, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0));
        RIGHT_CONFIG.encoder.positionConversionFactor(Constants.constants_Elevator.ELEVATOR_GEAR_RATIO);
        RIGHT_CONFIG.inverted(constants_Elevator.RIGHT_INVERTED);
        RIGHT_CONFIG.idleMode(IdleMode.kBrake);

        ROLLER_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.00, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0));
        ROLLER_CONFIG.encoder.positionConversionFactor(Constants.constants_Elevator.ELEVATOR_GEAR_RATIO);
        ROLLER_CONFIG.inverted(constants_Elevator.ROLLER_INVERTED);
        ROLLER_CONFIG.idleMode(IdleMode.kBrake);

        ELE_LEFT= new SparkMax(MAP_ELEVATOR.ELEVATOR_LEFT, MotorType.kBrushless);
        ELE_LEFT_ENCODER = ELE_LEFT.getEncoder();
        ELE_LEFT_PID = ELE_LEFT.getClosedLoopController();
        ELE_LEFT.configure(LEFT_CONFIG.inverted(constants_Elevator.LEFT_INVERTED), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        ELE_RIGHT= new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_RIGHT_ENCODER = ELE_RIGHT.getEncoder();
        ELE_RIGHT_PID = ELE_RIGHT.getClosedLoopController();
        ELE_RIGHT.configure(RIGHT_CONFIG.inverted(constants_Elevator.RIGHT_INVERTED), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        ELE_ROLLER = new SparkMax(MAP_ELEVATOR.ELEVATOR_ROLLERS, MotorType.kBrushless);
        ELE_ROLLER_ENCODER = ELE_ROLLER.getEncoder();
        ELE_ROLLER_PID = ELE_ROLLER.getClosedLoopController();
        ELE_ROLLER.configure(ROLLER_CONFIG, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public double getPosition()
    {
        return ELE_LEFT_ENCODER.getPosition();
    }
    
    public double getDistance()
    {
        return CANrange.getDistance().getValueAsDouble()* 39.3701; //Meters to Inches
    }
    
    public boolean getGamePieceStored()
    {
    //    return getDistance() < 10;
        return testBool;
    }
    
    public void setElevatorPosition(ElevatorPositionGroup position)
    {
        if(position.elevatorPositionInches >= 0)
        {
            ELE_LEFT_PID.setReference(position.elevatorPositionInches, ControlType.kPosition);
            ELE_RIGHT_PID.setReference(position.elevatorPositionInches, ControlType.kPosition);
        }
        if(position.elevatorRollersRPM >= 0)
        {
            ELE_ROLLER_PID.setReference(position.elevatorRollersRPM, ControlType.kVelocity);
        }
    }

    public boolean isElevatorInPosition(ElevatorPositionGroup position)
    {
        return Math.abs(getPosition() - position.elevatorPositionInches) < constants_Elevator.ELEVATOR_TOLERANCE;
    }

    public double getElevatorRollerRPM()
    {
        return ELE_ROLLER_ENCODER.getVelocity();
    }
     @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Coral Detection", getGamePieceStored());
        SmartDashboard.putNumber("Elevator Position", getPosition());
        
    }

}
    
    
    
