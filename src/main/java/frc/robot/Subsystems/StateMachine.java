// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.constStateMachine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Drive;
import frc.robot.Commands.States.ClimbingState;
import frc.robot.Commands.States.IntakingState;
import frc.robot.Commands.States.NoneState;
import frc.robot.Commands.States.PrepTargetState;
import frc.robot.Commands.States.PrepVision;
import frc.robot.Commands.States.ScoringState;
import frc.robot.Commands.States.StoreCoralState;

import frc.robot.Util.Constants.*;



public class StateMachine extends SubsystemBase {
  public static RobotState currentState;
  public static TargetState currentTargetState;

  /** Creates a new StateMachine. */
  public StateMachine() {
    currentState = RobotState.NONE;
    currentTargetState = TargetState.PREP_NONE;
  }

  public void setRobotState(RobotState robotState) 
  {
    currentState = robotState;
  }

  public void setTargetState(TargetState targetState) 
  {
    currentTargetState = targetState;
  }

  public RobotState getRobotState() 
  {
    return currentState;
  }

  public TargetState getTargetState() 
  {
    return currentTargetState;
  }

 
  public Command tryState(RobotState desiredState, StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights) {


    switch (desiredState) {
      case NONE:
        switch (currentState) {
          case INTAKING:
          case SCORING:
          case NONE:
            return new NoneState(desiredState, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
        }
        break;
      case INTAKING:
        switch (currentState) {
          case NONE:
          case SCORING: //TODO need to find out if you were in the combo state before scoring so that you can go to algae instead of none
            return new IntakingState();
        }
        break;

      case STORE_CORAL:
        switch (currentState) {
          case STORE_CORAL:
          case INTAKING:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_L4:
          case PREP_VISION:
          case PREP_NONE:
            return new StoreCoralState();
        }
        break;

      case CLIMBING:
        switch (currentState) {
          case NONE:
          case STORE_CORAL:
          case PREP_NONE:
            return new ClimbingState();
        }
        break;

      case SCORING:
        switch(currentState)
        {
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_L4:
          case PREP_VISION:
          case PREP_NONE:
          case CLIMBING:
          case SCORING:
            return new ScoringState();
        }
        break;

      case PREP_L1:
        switch (currentState) {
          case NONE:
          case STORE_CORAL:
          case INTAKING:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_L4:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_Elevator, s_StateMachine, s_Lights, TargetState.PREP_L1);
        }
        break;

      case PREP_L2:
        switch (currentState) {
          case NONE:
          case STORE_CORAL:
          case INTAKING:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_L4:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_Elevator, s_StateMachine, s_Lights, TargetState.PREP_L2);
        }
        break;
        
      case PREP_L3:
        switch (currentState) {
          case NONE:
          case STORE_CORAL:
          case INTAKING:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_L4:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_Elevator, s_StateMachine, s_Lights, TargetState.PREP_L3);
        }
        break;

      case PREP_L4:
        switch (currentState) {
          case NONE:
          case STORE_CORAL:
          case INTAKING:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_L4:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_Elevator, s_StateMachine, s_Lights, TargetState.PREP_L4);
        }
        break;
      
      
    }
    return Commands.print("HAWK TUAH :O INVALID STATE GIVEN");
  }

  public Command tryTargetState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Lights s_Lights) 
  {
    if (currentTargetState.equals(TargetState.PREP_VISION))
    {
      return new PrepVision(s_StateMachine, c_Drive, s_Elevator, s_Lights);
    }
    return new PrepTargetState(s_Elevator, s_StateMachine, s_Lights, currentTargetState);
  }

    /**
   * Determines if our current robot state is also a target state.
   */
  public boolean isCurrentStateTargetState()
  {
    return isGivenStateTargetState(currentState);
  }

  public boolean isGivenStateTargetState(RobotState givenState)
  {
    for (TargetState targetState : TargetState.values()) {
      RobotState possibleRobotState = constants_StateMachine.TARGET_TO_ROBOT_STATE.get(targetState);
      if (givenState.equals(possibleRobotState)) {
        return true;
      }
    }

    return false;
  }

  public static enum RobotState {
    NONE,
    INTAKING,
    STORE_CORAL,
    CLIMBING,
    SCORING,
    PREP_NONE,
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4,
    PREP_VISION
  }

  public static enum TargetState
  {
    PREP_NONE,
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4,
    PREP_VISION
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());
  }
}
