package frc.robot.Subsystems.Drive;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class Module extends SubsystemBase
{
    



  public static TalonFX driveMotor;
  public static VelocityVoltage velocityRequest;
  public static MotionMagicVelocityVoltage motionMagicRequest;
  public static TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  public static NeutralOut neutralOut;
  static Slot0Configs driveGains;
  
  public static LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.55);
  public static Current slipCurrent = edu.wpi.first.units.Units.Amps.of(120);
  public static ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  public static SparkMax steerMotor;
  public static SparkClosedLoopController steerPIDController;
  public static RelativeEncoder steerEncoder;
  public static SparkBaseConfig steerGains;

  public static Rotation2d zeroRotation;
  public static CANcoder absoluteEncoder;
  public boolean absoluteReversed;


  
  // Inputs from drive motor
  public static StatusSignal<Angle> drivePosition;
  // public static Queue<Double> drivePositionQueue;
  public static StatusSignal<AngularVelocity> driveVelocity;
  public static StatusSignal<Voltage> driveAppliedVolts;
  public static StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  public static StatusSignal<Angle> steerAbsolutePosition;
  public static StatusSignal<Angle> steerPosition;
  // public static Queue<Double> turnPositionQueue;
  public static StatusSignal<AngularVelocity> turnVelocity;
  public static StatusSignal<Voltage> turnAppliedVolts;
  public static StatusSignal<Current> turnCurrent;

  // Voltage control requests
  public static VoltageOut voltageRequest = new VoltageOut(0);
  // private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  public static VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  
  
  public Module(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderID, Rotation2d zeroRotation, boolean absoluteReversed)
  {
    this.absoluteReversed = absoluteReversed;

    driveMotor = new TalonFX(driveNum);
    driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);

    steerMotor = new SparkMax(steerNum, MotorType.kBrushless);
    steerGains = new SparkMaxConfig()
        .apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
    steerPIDController = steerMotor.getClosedLoopController();

    //TODO Config the abs encoder

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
  


  public SwerveModulePosition getSteerPosition()
  {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(steerEncoder.getPosition()));
  }
  
  public Rotation2d steerR2d = Rotation2d.fromDegrees(getSteerPosition().angle.getDegrees());

  public SwerveModuleState getSteerState()
  {
    return new SwerveModuleState(getDriveVelocity(), steerR2d);
  }
  public double getABSPosition()
  {
    return (absoluteEncoder.getAbsolutePosition().getValueAsDouble()-zeroRotation.getDegrees()) * 360;
  }
  public double getDrivePosition()
  {
      return driveMotor.getPosition().getValueAsDouble();
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

      
  //This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
  //location
  public void setDesiredState(SwerveModuleState state) 
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
    // state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getSteerState().angle.getDegrees()));
    state.cosineScale(steerR2d);
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
