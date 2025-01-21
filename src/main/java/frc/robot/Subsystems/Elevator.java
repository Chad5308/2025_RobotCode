package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RobotMap.MAP_ELEVATOR;

public class Elevator extends SubsystemBase
{
    public TalonFX ELE_LEFT;
    public TalonFX ELE_RIGHT;
    
    public MotionMagicVelocityVoltage motionMagicRequest;
    // public TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    public NeutralOut neutralOut;
    public Slot0Configs elevatorConfigs;
    public ClosedLoopOutputType elevatorClosedLoopOutput = ClosedLoopOutputType.Voltage;


    public SparkMax ELE_ARM;
    public RelativeEncoder ELE_ARM_ENCODER;
    public SparkClosedLoopController ELE_ARM_PID;

    public SparkBaseConfig config;


    public Elevator()
    {
        ELE_LEFT = new TalonFX(MAP_ELEVATOR.ELEVATOR_LEFT);
        ELE_RIGHT = new TalonFX(MAP_ELEVATOR.ELEVATOR_RIGHT);

        elevatorConfigs = new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0.5).withKV(0.2);
        
        ELE_LEFT.getConfigurator().apply(elevatorConfigs);
        ELE_RIGHT.getConfigurator().apply(elevatorConfigs);

        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_ARM = new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_ARM_ENCODER = ELE_ARM.getEncoder();
        ELE_ARM_PID = ELE_ARM.getClosedLoopController();
        ELE_ARM.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }


    public double getPosition()
    {
        return 0;
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
    
}
