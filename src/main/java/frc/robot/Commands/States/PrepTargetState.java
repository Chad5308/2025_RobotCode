// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;
import frc.robot.Util.Constants.*;


public class PrepTargetState extends Command {
  StateMachine s_StateMachine;
  Elevator s_Elevator;
  Lights s_Lights;
  TargetState desiredTargetState;
  AlgaeRollers s_Rollers;
  boolean ALGAE_STATE;

  ElevatorPositionGroup desiredElevatorPosition;
  boolean elevatorWasUp;

  /** Creates a new PrepTargetState. */
  public PrepTargetState(StateMachine s_StateMachine, Elevator s_Elevator, AlgaeRollers s_Rollers, Lights s_Lights, TargetState desiredTargetState) {
    this.s_StateMachine = s_StateMachine;
    this.s_Elevator = s_Elevator;
    this.s_Lights = s_Lights;
    this.desiredTargetState = desiredTargetState;
    this.s_Rollers = s_Rollers;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_StateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState desiredRobotState = constants_StateMachine.TARGET_TO_ROBOT_STATE.get(desiredTargetState);
    RobotState currentRobotState = s_StateMachine.getRobotState();

    if (currentRobotState.equals(RobotState.ALGAE) || currentRobotState.equals(RobotState.CORAL) || currentRobotState.equals(RobotState.COMBO) || s_StateMachine.isCurrentStateTargetState()) {
      s_StateMachine.setRobotState(desiredRobotState);
    }

    desiredElevatorPosition = constants_StateMachine.TARGET_TO_PRESET_GROUP.get(desiredTargetState);


    if(s_StateMachine.getTargetState() == TargetState.PREP_ALGAE)
    {
      s_Elevator.setElevatorPosition(constants_Elevator.CORAL);
    } else
    { 
      if(desiredElevatorPosition == constants_Elevator.PREP_L1)
      {
        s_Elevator.setElevatorPosition(constants_Elevator.PREP_L1);
        Commands.waitSeconds(0.1);
        s_Elevator.setElevatorPosition(desiredElevatorPosition);
      }
      s_Elevator.setElevatorPosition(desiredElevatorPosition);      
    }

    
  

    switch(desiredTargetState)
    {
      case PREP_ALGAE:
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_PREP_ALGAE_PATTERN);
        break;
      case PREP_NONE:
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_PREP_NONE_PATTERN);
        break;
      case PREP_L1:
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_PREP_L1_PATTERN);
        break;
      case PREP_L2:
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_PREP_L2_PATTERN);
        break;
      case PREP_L3:
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_PREP_L3_PATTERN);
        break;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if(s_StateMachine.getTargetState() == TargetState.PREP_ALGAE)
    {
      s_Elevator.setElevatorPosition(constants_Elevator.CORAL);
    } else { 
      s_Elevator.setElevatorPosition(desiredElevatorPosition);
    }
    //When the command ends we try and move again just incase
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(s_StateMachine.getTargetState() == TargetState.PREP_ALGAE)
    {
      return s_Rollers.isRollersInPosition(60);
    } else { 
      return s_Elevator.isElevatorInPosition(desiredElevatorPosition);
    }
    //should return if the elevator and arm are in tolerance of where we want them to be
  }
}
