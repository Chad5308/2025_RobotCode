// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Subsystems.Drive;

import static frc.robot.Util.DriveUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.Constants.constants_Drive;
import frc.robot.Util.Constants.constants_Module;
import frc.robot.Util.Constants.constants_Sim;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIODrive implements ModuleIO {


public TalonFX driveKraken;
  public NeutralModeValue neutralModeValue;
  public Slot0Configs driveConfigs;
  public FeedbackConfigs driveFeedbackConfigs;

  private final SparkMax turnSpark;
  public SparkClosedLoopController turnPIDController;
  public RelativeEncoder turnEncoder;
  public SparkBaseConfig turnConfig;

  public double absOffset;
  public CANcoder absoluteEncoder;
  public CANcoderConfiguration CANConfig;
  public boolean absoluteReversed;

    // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);


        // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
//   private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIODrive(int module) {

    driveKraken = new TalonFX((module+1));
    driveConfigs = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
    driveFeedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(constants_Module.DRIVE_GEAR_RATIO*constants_Module.DRIVE_ROT_2_METER);
    neutralModeValue = NeutralModeValue.Brake;
    driveKraken.getConfigurator().apply(driveConfigs);
    driveKraken.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(80));
    driveKraken.getConfigurator().apply(driveFeedbackConfigs, 5);
    driveKraken.getConfigurator().apply(new MotorOutputConfigs().withInverted(switch(module)
    {
        case 0 -> constants_Drive.FL_DRIVE_ENCODER_REVERSED;
        case 1 -> constants_Drive.FR_DRIVE_ENCODER_REVERSED;
        case 2 -> constants_Drive.BL_DRIVE_ENCODER_REVERSED;
        case 3 -> constants_Drive.BR_DRIVE_ENCODER_REVERSED;
        default -> false;
    }?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive).withNeutralMode(neutralModeValue));
  
    driveKraken.setPosition(0.0);


    turnConfig = new SparkMaxConfig()
    .apply(new ClosedLoopConfig().pidf(0.0225, 0.000001, 0.0, 0.0, ClosedLoopSlot.kSlot0).positionWrappingEnabled(true).positionWrappingInputRange(-179.9999999, 180));
    turnConfig.encoder.positionConversionFactor(constants_Module.STEER_TO_RAD).velocityConversionFactor(constants_Module.STEER_RPM_2_RAD_PER_SEC);
    turnConfig.inverted(switch(module)
    {
        case 0 -> constants_Drive.FL_STEER_ENCODER_REVERSED;
        case 1 -> constants_Drive.FR_STEER_ENCODER_REVERSED;
        case 2 -> constants_Drive.BL_STEER_ENCODER_REVERSED;
        case 3 -> constants_Drive.BR_STEER_ENCODER_REVERSED;
        default -> false;
    }).idleMode(IdleMode.kBrake).voltageCompensation(12.25);


    turnSpark = new SparkMax((module+1), MotorType.kBrushless);
    turnEncoder = turnSpark.getEncoder();
    turnPIDController = turnSpark.getClosedLoopController();
    turnSpark.configure(turnConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnEncoder.setPosition(0);

    absOffset =
        switch (module) {
          case 0 -> constants_Drive.FL_OFFSET;
          case 1 -> constants_Drive.FR_OFFSET;
          case 2 -> constants_Drive.BL_OFFSET;
          case 3 -> constants_Drive.BR_OFFSET;
          default -> 0;
        };


    CANConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absOffset).withAbsoluteSensorDiscontinuityPoint(0.5));
    CANConfig.MagnetSensor.SensorDirection =
        switch(module)
    {
        case 0 -> constants_Drive.FL_DRIVE_ABSOLUTE_ENCODER_REVERSED;
        case 1 -> constants_Drive.FR_DRIVE_ABSOLUTE_ENCODER_REVERSED;
        case 2 -> constants_Drive.BL_DRIVE_ABSOLUTE_ENCODER_REVERSED;
        case 3 -> constants_Drive.BR_DRIVE_ABSOLUTE_ENCODER_REVERSED;
        default -> false;
    }
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    absoluteEncoder = new CANcoder((module+1), new CANBus());
    absoluteEncoder.getConfigurator().apply(CANConfig);


    

    // Create drive status signals
    drivePosition = driveKraken.getPosition();
    drivePositionQueue =
        OdometryThread.getInstance().registerSignal(drivePosition);
    driveVelocity = driveKraken.getVelocity();
    driveAppliedVolts = driveKraken.getMotorVoltage();
    driveCurrent = driveKraken.getStatorCurrent();

     // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        constants_Sim.odometryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveKraken);



    // Create odometry queues
    timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    turnPositionQueue =
        OdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs)
  {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionM = drivePosition.getValueAsDouble();
    inputs.driveVelocityMPS = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();


    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(getABSPosition()));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsM =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

 @Override
  public void setDriveOpenLoop(double output) {
    driveKraken.setControl(
        voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

 @Override
  public void setDriveVelocity(double velocityMPS) {
    double velocityRotPerSec = velocityMPS * constants_Module.DRIVE_MPS_2_ROT_PER_SEC;
    driveKraken.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.getRadians(), constants_Sim.TURN_PID_MIN_INPUT, constants_Sim.TURN_PID_MAX_INPUT);
    turnPIDController.setReference(setpoint, ControlType.kPosition);
  }


   public double getABSPosition()
  {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble()*(2*Math.PI); //Radians
  }
  

  @Override
  public void resetWheels() 
  {
    turnEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      turnPIDController.setReference(0, ControlType.kPosition);
    } catch (Exception e){}
  }

  @Override
  public void resetWheelsRight() 
  {
    turnEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      turnPIDController.setReference(90, ControlType.kPosition);
    } catch (Exception e){}
  }

}
