package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

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

    public Elevator()
    {
        ELE_LEFT = new TalonFX(MAP_ELEVATOR.ELEVATOR_LEFT);
        ELE_RIGHT = new TalonFX(MAP_ELEVATOR.ELEVATOR_RIGHT);

        elevatorConfigs = new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0.5).withKV(0.2);
        
        ELE_LEFT.getConfigurator().apply(elevatorConfigs);
        ELE_RIGHT.getConfigurator().apply(elevatorConfigs);
        
    }


    public double getPosition()
    {
        return 0;
    }
    
    
}
