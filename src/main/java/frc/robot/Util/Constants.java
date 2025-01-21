// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class constants_Drive {

    public static final Measure<DistanceUnit> WHEEL_RADIUS = edu.wpi.first.units.Units.Inches.of(1.5);
    // public static final double wheelRadius = Units.inchesToMeters(1.5);
    public static final double COF = 1.2;
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);
      // Distance between right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(23.75);
      // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //front left
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //front right
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), //back left
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)); //back right

    
    //TODO Test and input all module offsets
    public static final Rotation2d FL_DEGREES = Rotation2d.fromDegrees(0);
    public static final Rotation2d FR_DEGREES = Rotation2d.fromDegrees(0);
    public static final Rotation2d BL_DEGREES = Rotation2d.fromDegrees(0);
    public static final Rotation2d BR_DEGREES = Rotation2d.fromDegrees(0);

    //TODO Invert any motor to match controller output
    public static final boolean FL_STEER_ENCODER_REVERSED = false;
    public static final boolean FR_STEER_ENCODER_REVERSED = false;
    public static final boolean BL_STEER_ENCODER_REVERSED = false;
    public static final boolean BR_STEER_ENCODER_REVERSED = false;

    public static final boolean FL_DRIVE_ENCODER_REVERSED = true;
    public static final boolean FR_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BL_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BR_DRIVE_ENCODER_REVERSED = false;

    public static final boolean FL_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FR_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

    public static final double MAX_SPEED_METERS_PER_SEC = 6.949; //6.949 for Swerve X, 4.60248 for sd
    public static final LinearVelocityUnit MAX_TESTED_SPEED = LinearVelocityUnit.combine(edu.wpi.first.units.Units.Meters.of(4.5).unit(), edu.wpi.first.units.Units.Seconds.of(1).unit());
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_SPEED_METERS_PER_SEC/(TRACK_WIDTH/2);

    //For limiting speed while driving
    public static final double TELEDRIVE_MAX_SPEED_METERS_PER_SEC = MAX_SPEED_METERS_PER_SEC / 1.0;
    public static final double TELEDRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_ANGULAR_SPEED_RAD_PER_SEC / 1.0;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC = 2.0;
    public static final double TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC = 0.75;
  }
  
  public static final class constants_Module {
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 4.59 / 1; //4.59 for Swerve X, 6.75 for sds
    public static final double STEER_MOTOR_GEAR_RATIO = 13.3714 / 1; //13.3714 for Swerve X, 12.8 for sds
    public static final double DRIVE_ENCODER_ROT_2_METER = 1/16.0344; //Not sure try 1/16.0344, 1/23.58 for sds

    // public static final double TURNING_CONVERSION_FACTOR_2_DEG =  28.25;
    public static final double DRIVE_ENCODER_RPM_2_METERS_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
    // public static final double TURNING_ENCODER_RPM_2_DEG_PER_SEC = TURNING_CONVERSION_FACTOR_2_DEG / 60;

    public static final double P_TURNING = 0.0075;
    public static final double I_TURNING = 0.0;
    public static final double D_TURNING = 0.75;
    public static final double FF_TURNING = 0;

    public static final double S_DRIVE = 0.4;
    public static final double V_DRIVE = 0.124;
    public static final double A_DRIVE = 0.1;
    public static final double P_DRIVE = 0.1;
    public static final double I_DRIVE = 0.1;

    public static final double MODULE_RADIUS = Units.inchesToMeters(Constants.constants_Drive.TRACK_WIDTH/2); //measured from center of robot to furthest module.
  }

  public static final class constants_Elevator
  {




    public static final ElevatorPositionGroup PREP_NONE = new ElevatorPositionGroup(edu.wpi.first.units.Units.Degrees.of(0), edu.wpi.first.units.Units.Inches.of(0));
    public static final ElevatorPositionGroup PREP_L1 = new ElevatorPositionGroup(edu.wpi.first.units.Units.Degrees.of(0), edu.wpi.first.units.Units.Inches.of(0));
    public static final ElevatorPositionGroup PREP_L2 = new ElevatorPositionGroup(edu.wpi.first.units.Units.Degrees.of(0), edu.wpi.first.units.Units.Inches.of(0));
    public static final ElevatorPositionGroup PREP_L3 = new ElevatorPositionGroup(edu.wpi.first.units.Units.Degrees.of(0), edu.wpi.first.units.Units.Inches.of(0));
    public static final ElevatorPositionGroup PREP_L4 = new ElevatorPositionGroup(edu.wpi.first.units.Units.Degrees.of(0), edu.wpi.first.units.Units.Inches.of(0));

  }

  public static class ElevatorPositionGroup
  {
    public Measure<AngleUnit> armAngle; //Degrees
    public Measure<DistanceUnit> elevatorPosition; //Inches from ground

    public ElevatorPositionGroup(Measure<AngleUnit> armAngle, Measure<DistanceUnit> elevatorPosition)
    {
      this.armAngle = armAngle;
      this.elevatorPosition = elevatorPosition;
    }
  }



  public static final class constants_OI {
    public static final int OP_CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.09;
    public static final int LEFT_STICK_PORT = 1;
    public static final int RIGHT_STICK_PORT = 2;
  }
 

  
  public static final class constants_AprilTags{ //[id num, height in inches, coordinate x, coordinate y, heading] inches to meters; use field manual image for id reference
    public static final double[] RED_CS_X = {1, 1.4859, 16.6992, 0.6553, 126};
    public static final double[] RED_CS_Y = {2, 1.4859, 16.6992, 7.4048, 234};
    public static final double[] RED_PROCESSOR = {3, 1.3017, 11.6087, 8.0641, 270};
    public static final double[] RED_SIDE_BLUE_BARGE = {4, 1.8678, 9.2800, 6.1340, 0};
    public static final double[] RED_SIDE_RED_BARGE = {5, 1.8678, 9.2800, 1.9148, 0};
    public static final double[] RED_REEF_KL = {6, 0.3081, 13.4742, 3.3074, 300};
    public static final double[] RED_REEF_AB = {7, 0.3081, 13.8934, 4.0262, 0};
    public static final double[] RED_REEF_CD = {8, 0.3081, 13.4742, 4.7449, 60};
    public static final double[] RED_REEF_EF = {9, 0.3081, 12.6464, 4.7449, 120};
    public static final double[] RED_REEF_GH = {10, 0.3081, 12.2194, 4.0262, 180};
    public static final double[] RED_REEF_IJ = {11, 0.3081, 12.6464, 3.3074, 240};
    public static final double[] BLUE_CS_Y = {12, 1.4859, 0.8507, 0.6553, 54};
    public static final double[] BLUE_CS_X = {13, 1.4859, 0.8507, 7.4048, 306};
    public static final double[] BLUE_PROCESSOR = {14, 1.8678, 8.2744, 6.1340, 0};
    public static final double[] BLUE_SIDE_BLUE_BARGE = {15, 1.8678, 8.2744, 1.9148, 180};
    public static final double[] BLUE_SIDE_RED_BARGE = {16, 1.3017, 5.9915, -0.0004, 90};
    public static final double[] BLUE_REEF_CD = {17, 0.3081, 4.0734, 3.3074, 240};
    public static final double[] BLUE_REEF_AB = {18, 0.3081, 3.6571, 4.0262, 0};
    public static final double[] BLUE_REEF_KL = {19, 0.3081, 4.0734, 4.7449, 120};
    public static final double[] BLUE_REEF_IJ = {20, 0.3081, 4.9068, 4.7449, 180};
    public static final double[] BLUE_REEF_GH = {21, 0.3081, 5.3246, 5.3246, 0};
    public static final double[] BLUE_REEF_EF = {22, 0.3081, 4.9068, 3.3074, 300};

    
  }

  public static final class constants_Limelight{
    public static final double THETA_P = 4;
    public static final double THETA_I = 0.0002;
    public static final double THETA_D = 0;

    public static final double LINEAR_P = 1.25;
    public static final double LINEAR_I = 0.001;
    public static final double LINEAR_D = 0.05;

    public static final double ANGLE_CORAL = -15;
    public static final double DISTANCE_FORWARD_CORAL = 5; //inches
    public static final double DISTANCE_RIGHT_CORAL = -3; //inches
    public static final double HEIGHT_CORAL = 23.75; //inches

    public static final double ANGLE_TAGS = 15;
    public static final double DISTANCE_FORWARD_TAGS = 3; //inches
    public static final double DISTANCE_RIGHT_TAGS = -3; //inches
    public static final double HEIGHT_TAGS = 24.5; //inches
  }


  public static final class constants_Auto {
    public static final double MAX_SPEED_METERS_PER_SEC = constants_Drive.MAX_SPEED_METERS_PER_SEC/2;//0.5;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQRD = constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC/2;//0.25;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =  constants_Drive.TELEDRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC;
    public static final double MAX_ANGULAR_ACCEL_UNITS_PER_SEC = constants_Drive.TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC;

    public static  double P_TRANSLATION = 3.0;
    public static  double I_TRANSLATION = 0.1;
    public static  double D_TRANSLATION = 2;

    public static final double P_THETA = 4.5;
    public static final double I_THETA = 0.1;
    public static final double D_THETA = 0;


    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_RAD_PER_SEC,
                    MAX_ANGULAR_ACCEL_UNITS_PER_SEC);
    public static final TrapezoidProfile.Constraints LINEAR_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_SPEED_METERS_PER_SEC,
                MAX_ACCELERATION_METERS_PER_SECOND_SQRD
            );
  }


  public static final class constants_StateMachine
  {
    public static final Map<TargetState, RobotState> TARGET_TO_ROBOT_STATE = new HashMap<TargetState, RobotState>();

    static
    {
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_NONE, RobotState.PREP_NONE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_INTAKE, RobotState.INTAKING);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L1, RobotState.PREP_L1);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L2, RobotState.PREP_L2);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L3, RobotState.PREP_L3);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L4, RobotState.PREP_L4);
    }

    public static Map<TargetState, ElevatorPositionGroup> TARGET_TO_PRESET_GROUP = new HashMap<TargetState, ElevatorPositionGroup>();

    static
    {
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_NONE, constants_Elevator.PREP_NONE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L1, constants_Elevator.PREP_L1);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L2, constants_Elevator.PREP_L2);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L3, constants_Elevator.PREP_L3);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L4, constants_Elevator.PREP_L4);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_INTAKE, constants_Elevator.PREP_NONE);
    }
  }
}