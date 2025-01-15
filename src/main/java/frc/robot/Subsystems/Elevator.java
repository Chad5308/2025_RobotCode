package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RobotMap.MAP_ELEVATOR;

public class Elevator extends SubsystemBase
{
    public SparkMax ELE_LEFT;
    public RelativeEncoder ELE_LEFT_ENCODER;
    public SparkClosedLoopController ELE_LEFT_PID;

    public SparkMax ELE_RIGHT;
    public RelativeEncoder ELE_RIGHT_ENCODER;
    public SparkClosedLoopController ELE_RIGHT_PID;

    public SparkBaseConfig config;

    public Elevator()
    {
        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));

        ELE_LEFT = new SparkMax(MAP_ELEVATOR.ELEVATOR_LEFT, MotorType.kBrushless);
        ELE_LEFT_ENCODER = ELE_LEFT.getEncoder();
        ELE_LEFT_PID = ELE_LEFT.getClosedLoopController();
        ELE_LEFT.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ELE_RIGHT = new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_RIGHT_ENCODER = ELE_RIGHT.getEncoder();
        ELE_RIGHT_PID = ELE_RIGHT.getClosedLoopController();
        ELE_RIGHT.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public double getPosition()
    {

        return 0;
    }
    
    
}
