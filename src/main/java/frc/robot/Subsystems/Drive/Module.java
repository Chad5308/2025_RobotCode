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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

  public Rotation2d absOffset;
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

  
  public Module(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderID, double absOffset, boolean absoluteReversed)
  {
    driveMotor = new TalonFX(driveNum);
    driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
    driveFeedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(constants_Module.DRIVE_GEAR_RATIO);
    driveMotor.getConfigurator().apply(driveGains);
    driveMotor.getConfigurator().apply(driveFeedbackConfigs, 5);
    
  
    
    this.absoluteReversed = absoluteReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderID, new CANBus());
    CANConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absOffset).withAbsoluteSensorDiscontinuityPoint(0.5));
    
    absoluteEncoder.getConfigurator().apply(CANConfig);
    

    steerMotor = new SparkMax(steerNum, MotorType.kBrushless);
    steerEncoder = steerMotor.getEncoder();
    steerGains = new SparkMaxConfig()
    .apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0).positionWrappingEnabled(true).positionWrappingInputRange(720, 1080));
    steerGains.encoder.positionConversionFactor(Constants.constants_Module.STEER_TO_DEGREES);
    steerGains.encoder.velocityConversionFactor(constants_Module.STEER__RPM_2_DEG_PER_SEC);
    steerPIDController = steerMotor.getClosedLoopController();
    steerMotor.configure(steerGains, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    steerGains.inverted(invertSteer);
    steerGains.idleMode(IdleMode.kBrake);
    
    resetEncoders();
  }
  
  public void stop()
  {
    driveMotor.set(0);
    steerMotor.set(0);
  }
  
  public void resetEncoders() 
  {
    driveMotor.setPosition(0);
    steerEncoder.setPosition(0);
  }
  
//Drive Methods
  public double getDrivePosition()
  {
      return driveMotor.getPosition().getValueAsDouble() * constants_Module.DRIVE_ROT_2_METER;
  }
  public double getDriveVelocity()
  {
    return driveMotor.getVelocity().getValueAsDouble();
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
  
  
  //Steer Methods
  public double getPosition()
  {
    return steerEncoder.getPosition();
  }
  public double getABSPosition()
  {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); //  * 360 to convert to degrees
    return (angle  * (absoluteReversed ? -1 : 1) ) % 720;
  }
  public SwerveModuleState getModuleState()
  {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getPosition()));
  }
  public SwerveModulePosition getModulePosition()
  {
    return new SwerveModulePosition(getDrivePosition(), getModuleState().angle);
  }


      
  //This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
  //location
  public void setDesiredState(SwerveModuleState state) 
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
    state.optimize(state.angle);
    state.cosineScale(state.angle);
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
    } catch (Exception e){}
  }
    
}
