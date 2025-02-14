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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Util.Constants;
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

    public DutyCycleEncoder absoluteEncoder;    

    //TODO Zeroing the elevator and converting to inches
    /*
     * 1. when the robot is off move the elevator all the way down
     * 2. Put a mark somewhere to indicate the point on the shoot where the coral comes out, right in the middle of the where the coral would be at the end of the shoot, when the elevator is at this point the RELATIVE ENCODERS should read 0
     * 3. move the elevator up to the max height of travel
     * 4. Make sure all of the 3 encoders are increasing positivly when moving upwards, if not, adjust the ones that arent moving the right way by flipping their coresponding "Inverted" boolean in the Constants_Elevator file. Do the same for the roller, when the wheel is rotating such that it spits out the coral that should be positive rotation
     * 5. Note down that point in the variable named "ELEVATOR_MAX_HEIGHT" under the constants_Elevator class
     * 6. Note down the rotations in the varible named "ELEVATOR_ROTATIONS_AT_MAX_HEIGHT" under the constants_Elevator class
     * 7. Push the code to the robot
     * 8. Bring the elevator all the way down without enabling
     * 9. Note down the number that "Elevator ABS Position" gives when all the way down in the variable "ABS_OFFSET"
     * 10. Push code again with the elevator all the way down and confirm that when starting from the bottom the 2 encoders read the same value
     */
    public Elevator()
    {
        absoluteEncoder = new DutyCycleEncoder(MAP_ELEVATOR.ELEVATOR_ABS_ENCODER_PWM_PORT);
        absoluteEncoder.setInverted(constants_Elevator.ABS_INVERTED);

        sensorConfigs = new CANrangeConfiguration();
        CANrange = new CANrange(MAP_ELEVATOR.ELEVATOR_CANRANGE);
        CANrange.getConfigurator().apply(sensorConfigs);

        LEFT_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(constants_Elevator.ELEVATOR_P, constants_Elevator.ELEVATOR_I, constants_Elevator.ELEVATOR_D, constants_Elevator.ELEVATOR_FF, ClosedLoopSlot.kSlot0));
        LEFT_CONFIG.encoder.positionConversionFactor(constants_Elevator.ELEVATOR_GEAR_RATIO);
        LEFT_CONFIG.inverted(constants_Elevator.LEFT_INVERTED);
        LEFT_CONFIG.idleMode(IdleMode.kBrake);

        RIGHT_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(constants_Elevator.ELEVATOR_P, constants_Elevator.ELEVATOR_I, constants_Elevator.ELEVATOR_D, constants_Elevator.ELEVATOR_FF, ClosedLoopSlot.kSlot0));
        RIGHT_CONFIG.encoder.positionConversionFactor(constants_Elevator.ELEVATOR_GEAR_RATIO);
        RIGHT_CONFIG.inverted(constants_Elevator.RIGHT_INVERTED);
        RIGHT_CONFIG.idleMode(IdleMode.kBrake);

        ROLLER_CONFIG = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(constants_Elevator.ROLLER_P, constants_Elevator.ROLLER_I, constants_Elevator.ROLLER_D, constants_Elevator.ROLLER_FF, ClosedLoopSlot.kSlot0));
        ROLLER_CONFIG.encoder.positionConversionFactor(constants_Elevator.ELEVATOR_GEAR_RATIO);
        ROLLER_CONFIG.inverted(constants_Elevator.ROLLER_INVERTED);
        ROLLER_CONFIG.idleMode(IdleMode.kBrake);

        ELE_LEFT= new SparkMax(MAP_ELEVATOR.ELEVATOR_LEFT_SPARKMAX, MotorType.kBrushless);
        ELE_LEFT_ENCODER = ELE_LEFT.getEncoder();
        ELE_LEFT_PID = ELE_LEFT.getClosedLoopController();
        ELE_LEFT.configure(LEFT_CONFIG.inverted(constants_Elevator.LEFT_INVERTED), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        ELE_RIGHT= new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT_SPARKMAX, MotorType.kBrushless);
        ELE_RIGHT_ENCODER = ELE_RIGHT.getEncoder();
        ELE_RIGHT_PID = ELE_RIGHT.getClosedLoopController();
        ELE_RIGHT.configure(RIGHT_CONFIG.inverted(constants_Elevator.RIGHT_INVERTED), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        ELE_ROLLER = new SparkMax(MAP_ELEVATOR.ELEVATOR_ROLLERS_SPARKMAX, MotorType.kBrushless);
        ELE_ROLLER_ENCODER = ELE_ROLLER.getEncoder();
        ELE_ROLLER_PID = ELE_ROLLER.getClosedLoopController();
        ELE_ROLLER.configure(ROLLER_CONFIG, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ELE_LEFT_ENCODER.setPosition(0);
        ELE_RIGHT_ENCODER.setPosition(0);
    }
    
    public double getPosition()
    {
        return (absoluteEncoder.get() * constants_Elevator.ELEVATOR_TO_INCHES) - constants_Elevator.ABS_OFFSET;
    }
    
    public double getDistance()
    {
        return CANrange.getDistance().getValueAsDouble()* 39.3701; //Meters to Inches
    }
    
    public boolean getGamePieceStored()
    {
       return getDistance() < constants_Elevator.ROLLER_SENSOR_TOLERANCE;
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
        // ELE_LEFT_ENCODER.setPosition(getPosition());
        // ELE_RIGHT_ENCODER.setPosition(getPosition());
        SmartDashboard.putBoolean("Coral Detection", getGamePieceStored());
        SmartDashboard.putNumber("Elevator ABS Position", getPosition());
        SmartDashboard.putNumber("Left Relative Position", ELE_LEFT_ENCODER.getPosition());
        SmartDashboard.putNumber("Right Relative Position", ELE_RIGHT_ENCODER.getPosition());
        SmartDashboard.putNumber("Roller Position", ELE_ROLLER_ENCODER.getPosition()); //TODO This can be deleted or commented out once it is found if this wheel rotates to push the game piece out is positive rotation
    }

}
    
    
    
