// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Drive;
import frc.robot.Commands.States.AlgaeState;
import frc.robot.Commands.States.CleanL2State;
import frc.robot.Commands.States.CleanL3State;
import frc.robot.Commands.States.ClimbingState;
import frc.robot.Commands.States.ComboState;
import frc.robot.Commands.States.IntakingAlgaeState;
import frc.robot.Commands.States.NoneState;
import frc.robot.Commands.States.PrepTargetState;
import frc.robot.Commands.States.ScoringState;
import frc.robot.Commands.States.SourceState;
import frc.robot.Commands.States.CoralState;

import frc.robot.Util.Constants.*;



public class StateMachine extends SubsystemBase {
  public static RobotState currentState;
  public static TargetState currentTargetState;

  /** Creates a new StateMachine. */
  public StateMachine()
  {
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


  //Our state machine diagram! -> https://www.tldraw.com/ro/7BgSxOY5qbTRsoqWiZpjX?d=v-325.-71.2543.1252.page
 
  public Command tryState(RobotState desiredState, StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights) {

    System.out.println(desiredState + "-------Desired");
    System.out.println(currentState + "-------current");


    switch (desiredState) {
      case NONE:
        switch (currentState)
        {
          case INTAKE_ALGAE:
          case SOURCE:
          case SCORING:
          case CLEAN_L2:
          case CLEAN_L3:
          case CORAL:
          case ALGAE:
          case NONE:
            if(s_Rollers.getGamePieceCollected() && s_Elevator.getGamePieceStored())
            {
              System.out.println("___________COMBO____________");
              return new ComboState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
            }else if(s_Elevator.getGamePieceStored())
            {
              System.out.println("___________CORAL____________");
              return new CoralState(s_StateMachine, s_Elevator, s_Vision, s_Lights);
            }else if(s_Rollers.getGamePieceCollected())
            {
              System.out.println("___________ALGAE____________");
              return new AlgaeState(s_StateMachine, s_Elevator, s_Rollers, s_Vision, s_Lights);
            }else
            {
              System.out.println("___________NONE____________");
              return new NoneState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
            }
        }
        break;

      case INTAKE_ALGAE:
        switch (currentState)
        {
          case NONE:
          case CORAL:
            return new IntakingAlgaeState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
        }
        break;
      
      case CORAL:
        switch (currentState)
        {
          
          case SOURCE:
            if(s_Rollers.getGamePieceCollected())
            {
              return new ComboState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
            }else
            {
              return new CoralState(s_StateMachine, s_Elevator, s_Vision, s_Lights);
            }
        }
        break;

      case ALGAE:
        switch (currentState)
        {
          case NONE:
          case SOURCE:
          case INTAKE_ALGAE:
          if(s_Elevator.getGamePieceStored())
          {
            return new ComboState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
          }else
          {
            return new AlgaeState(s_StateMachine, s_Elevator, s_Rollers, s_Vision, s_Lights);
          }
        }
        break;

      case SOURCE:
        switch(currentState)
        {
          case NONE:
          case ALGAE:
            return new SourceState(s_StateMachine, s_Elevator, s_Vision, s_Lights);
        }
        break;
      
      case CLEAN_L2:
        switch(currentState)
        {
          case NONE:
          case ALGAE:
          case CLEAN_L3:
          case CLEAN_L2:
            return new CleanL2State(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
        }
        break;

      case CLEAN_L3:
        switch(currentState)
        {
          case NONE:
          case ALGAE:
          case CLEAN_L2:
          case CLEAN_L3:
            return new CleanL3State(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
        }
        break;

      case CLIMBING:
        switch (currentState)
        {
          case NONE:
          case CORAL:
          case ALGAE:
          case COMBO:
          // case PREP_NONE:
            return new ClimbingState(s_StateMachine, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
        }
        break;

      case SCORING:
        switch(currentState)
        {
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_VISION:
          case PREP_NONE:
          case PREP_ALGAE:
          case SCORING:
            return new ScoringState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
        }
        break;

      case PREP_L1:
        switch (currentState)
        {
          case CORAL:
          case COMBO:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_StateMachine, s_Elevator, s_Rollers, s_Lights, currentTargetState);
        }
        break;

      case PREP_L2:
        switch (currentState)
        {
          case CORAL:
          case COMBO:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_StateMachine, s_Elevator, s_Rollers, s_Lights, currentTargetState);
        }
        break;
        
      case PREP_L3:
        switch (currentState)
        {
          case CORAL:
          case COMBO:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_VISION:
          case PREP_NONE:
            return new PrepTargetState(s_StateMachine, s_Elevator, s_Rollers, s_Lights, currentTargetState);
        }
        break;

      case PREP_ALGAE:
        switch(currentState)
        {
          case COMBO:
          case ALGAE:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_ALGAE:
          case PREP_NONE:
          case PREP_VISION:
            return new PrepTargetState(s_StateMachine, s_Elevator, s_Rollers, s_Lights, currentTargetState);
        }
        break;

      case PREP_NONE:
        switch (currentState)
        {
          case NONE:
          case PREP_L1:
          case PREP_L2:
          case PREP_L3:
          case PREP_NONE:
          case PREP_VISION:
          case PREP_ALGAE:
            return new PrepTargetState(s_StateMachine, s_Elevator, s_Rollers, s_Lights, currentTargetState);
        }
        break;
    }
    System.out.println(desiredState);
    return Commands.print("HAWK TUAH :O INVALID STATE GIVEN");
  
  }

  // public Command tryTargetState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Vision s_Vision, Lights s_Lights) 
  // {
  //   if (currentTargetState.equals(TargetState.PREP_VISION))
  //   {
  //     return new PrepVision(s_StateMachine, c_Drive, s_Vision, s_Lights);
  //   }
  //   return new PrepTargetState(s_StateMachine, s_Elevator, null, s_Lights, currentTargetState);
  // }

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
    INTAKE_ALGAE,
    SOURCE,
    CORAL,
    ALGAE,
    COMBO,
    CLEAN_L2,
    CLEAN_L3,
    CLIMBING,
    SCORING,
    PREP_NONE,
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_VISION,
    PREP_ALGAE
  }

  public static enum TargetState
  {
    NONE,
    INTAKE_ALGAE,
    SOURCE,
    CORAL,
    ALGAE,
    COMBO,
    CLEAN_L2,
    CLEAN_L3,
    CLIMBING,
    SCORING,
    PREP_NONE,
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_VISION,
    PREP_ALGAE
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());


    
  }
}
