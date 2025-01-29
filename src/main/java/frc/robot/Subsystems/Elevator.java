package frc.robot.Subsystems;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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


    public Elevator()
    {
        // ELE_LEFT = new TalonFX(MAP_ELEVATOR.ELEVATOR_LEFT);
        // ELE_RIGHT = new TalonFX(MAP_ELEVATOR.ELEVATOR_RIGHT);

        // ELE_CONFIG = new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0.5).withKV(0.2);
        // ELE_FEEDBACKCONFIG = new FeedbackConfigs().withRotorToSensorRatio(constants_Elevator.ELEVATOR_TO_INCHES);
        
        // ELE_LEFT.getConfigurator().apply(ELE_CONFIG);
        // ELE_LEFT.getConfigurator().apply(ELE_FEEDBACKCONFIG);

        // ELE_RIGHT.getConfigurator().apply(ELE_CONFIG);
        // ELE_RIGHT.getConfigurator().apply(ELE_FEEDBACKCONFIG);

        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_ARM = new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_ARM_ENCODER = ELE_ARM.getEncoder();
        ELE_ARM_PID = ELE_ARM.getClosedLoopController();
        ELE_ARM.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_LEFT= new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_LEFT_ENCODER = ELE_ARM.getEncoder();
        ELE_LEFT_PID = ELE_ARM.getClosedLoopController();
        ELE_LEFT.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        ELE_RIGHT= new SparkMax(MAP_ELEVATOR.ELEVATOR_RIGHT, MotorType.kBrushless);
        ELE_RIGHT_ENCODER = ELE_ARM.getEncoder();
        ELE_RIGHT_PID = ELE_ARM.getClosedLoopController();
        ELE_RIGHT.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);        
    }


    public double getPosition()
    {
        return ELE_LEFT_ENCODER.getPosition();
    }
    
    public boolean getGamePieceStored()
    {
        return testBool;
        //TODO Code this distance/proxy sensor in
    }

    // public void setElevatorPosition(ElevatorPositionGroup position)
    // {
    //     return ELE_LEFT_ENCODER.setPosition(positi);
    // }


     @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Coral Detection", getGamePieceStored());
    }

    
}
