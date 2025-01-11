// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.constStateMachine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class StateMachine extends SubsystemBase {
  public static RobotState currentState;
  public static TargetState currentTargetState;

  /** Creates a new StateMachine. */
  public StateMachine() {
    currentState = RobotState.NONE;
    currentTargetState = TargetState.PREP_NONE;
  }

  public void setRobotState(RobotState robotState) {
    currentState = robotState;
  }

  public void setTargetState(TargetState targetState) {
    currentTargetState = targetState;
  }

  public RobotState getRobotState() {
    return currentState;
  }

  public TargetState getTargetState() {
    return currentTargetState;
  }

  /**
   * Determines which command to run for a desired state depending on if our
   * current state.
   * 
   * @see <a
   *      href=https://www.tldraw.com/ro/DX06u039erL_iV6q0ARSn?d=v-1103.-1504.5212.2506.page>
   *      Our State Machine Diagram
   *      </a>
   * @param desiredState The state you would like to go to, which may not be
   *                     possible from your current state
   * @return The Command to run for that desired state
   */
  public Command tryState(RobotState desiredState, StateMachine s_StateMachine, Drive s_Drive, Lights s_Lights) {


    switch (desiredState) {
      case NONE:
        switch (currentState) {
          case NONE:
            return new NoneState(s_StateMachine, s_Lights);
        }
        break;
      // 

    }
    return Commands.print("ITS SO OVER D: Invalid State Provided :3");
  }

  public static enum RobotState {
    NONE,
    ALGAE,
    CORAL,
    COMBO,
    CLIMB,
    SCORECORAL,
    SCOREALGAE
  }

  public static enum TargetState {
    PREP_NONE,
    PREPLL1,
    PREPLL2    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());
  }
}
