package frc.robot.Subsystems.Drive;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.constants_Module;

public class Module extends SubsystemBase
{


  public TalonFX driveMotor;
  public VelocityVoltage velocityRequest;
  public MotionMagicVelocityVoltage motionMagicRequest;
  // public TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  public NeutralOut neutralOut;
  public Slot0Configs driveGains;
  public FeedbackConfigs driveFeedbackConfigs;
  
  public LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.55);
  public Current slipCurrent = edu.wpi.first.units.Units.Amps.of(120);
  public ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  public SparkMax steerMotor;
  public SparkClosedLoopController steerPIDController;
  public RelativeEncoder steerEncoder;
  public SparkBaseConfig steerGains;

  public Rotation2d zeroRotation;
  public CANcoder absoluteEncoder;
  public CANcoderConfiguration CANConfig;
  public boolean absoluteReversed;


  
  // Inputs from drive motor
  public StatusSignal<Angle> drivePosition;
  // public static Queue<Double> drivePositionQueue;
  public StatusSignal<AngularVelocity> driveVelocity;
  public StatusSignal<Voltage> driveAppliedVolts;
  public StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  public StatusSignal<Angle> steerAbsolutePosition;
  public StatusSignal<Angle> steerPosition;
  // public static Queue<Double> turnPositionQueue;
  public StatusSignal<AngularVelocity> turnVelocity;
  public StatusSignal<Voltage> turnAppliedVolts;
  public StatusSignal<Current> turnCurrent;

  // Voltage control requests
  public VoltageOut voltageRequest = new VoltageOut(0);
  // private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  public VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  
  public Module(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderID, Rotation2d zeroRotation, boolean absoluteReversed)
  {
    driveMotor = new TalonFX(driveNum);
    driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
    driveFeedbackConfigs = new FeedbackConfigs().withRotorToSensorRatio(constants_Module.DRIVE_ENCODER_ROT_2_METER);
    driveMotor.getConfigurator().apply(driveGains);
    driveMotor.getConfigurator().apply(driveFeedbackConfigs, 5);
    
    //TODO Find where to invert drive and steer motors in documentation
  
    
    this.absoluteReversed = absoluteReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderID, new CANBus());
    CANConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(zeroRotation.getDegrees()).withAbsoluteSensorDiscontinuityPoint(0.5));
    absoluteEncoder.getConfigurator().apply(CANConfig);
    

    steerMotor = new SparkMax(steerNum, MotorType.kBrushless);
    steerGains = new SparkMaxConfig()
    .apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0).positionWrappingEnabled(true));
    steerGains.encoder.positionConversionFactor(Constants.constants_Module.STEER_MOTOR_GEAR_RATIO);
    steerPIDController = steerMotor.getClosedLoopController();
    steerMotor.configure(steerGains, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    
    resetDriveEncoder();
  }
  
  public void stop()
  {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public void resetDriveEncoder() 
  {
    driveMotor.setPosition(0);
  }
  
//Drive Values
  public double getDrivePosition()
  {
      return driveMotor.getPosition().getValueAsDouble();
  }
  public double getDriveVelocity()
  {
    return driveMotor.getVelocity().getValueAsDouble();
  }
  
  //Steer Values
  public SwerveModulePosition getSteerPosition()
  {
    return new SwerveModulePosition(getDrivePosition(), getSteerR2d());
  }

  public Rotation2d getSteerR2d()
  {
    return Rotation2d.fromDegrees(getABSPosition());
  }
  // public Rotation2d steerR2d = Rotation2d.fromDegrees(0);

  
  public SwerveModuleState getSteerState()
  {
    return new SwerveModuleState(getDriveVelocity(), getSteerR2d());
  }
  public double getABSPosition()
  {
    return (absoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void getUpToSpeed(double velocity)
  {
    if(velocity <= 0.01)
    {
      setDriveNeutralOutput();
    }else
    {
      driveMotor.setControl(motionMagicRequest.withVelocity(velocity));
    }
  }


  public void setDriveNeutralOutput()
  {
    driveMotor.setControl(new NeutralOut()); //TODO see if this is coast or brake
  }

      
  //This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
  //location
  public void setDesiredState(SwerveModuleState state) 
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
    // state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getSteerState().angle.getDegrees()));
    state.cosineScale(getSteerR2d());
    driveMotor.set(state.speedMetersPerSecond / Constants.constants_Drive.MAX_SPEED_METERS_PER_SEC);
    steerPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
    
  }

  public void wheelFaceForward() 
  {
    steerEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      steerPIDController.setReference(0, ControlType.kPosition);
    } catch (Exception e) 
    {

    }
  }
    
}
