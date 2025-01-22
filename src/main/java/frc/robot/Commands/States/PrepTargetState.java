// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.States;

import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Util.Constants.ElevatorPositionGroup;
import frc.robot.Util.Constants.*;


public class PrepTargetState extends Command {
  StateMachine s_StateMachine;
  Elevator s_Elevator;
  Lights s_LEDs;
  TargetState desiredTargetState;

  ElevatorPositionGroup desiredElevatorPosition;
  boolean elevatorWasUp;

  /** Creates a new PrepTargetState. */
  public PrepTargetState(Elevator s_Elevator, StateMachine s_StateMachine, Lights s_LEDs, TargetState desiredTargetState) {
    this.s_StateMachine = s_StateMachine;
    this.s_Elevator = s_Elevator;
    this.s_LEDs = s_LEDs;
    this.desiredTargetState = desiredTargetState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_StateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState desiredRobotState = constants_StateMachine.TARGET_TO_ROBOT_STATE.get(desiredTargetState);
    RobotState currentRobotState = s_StateMachine.getRobotState();

    if (currentRobotState.equals(RobotState.INTAKE_ALGAE) || s_StateMachine.isCurrentStateTargetState()) {
      s_StateMachine.setRobotState(desiredRobotState);
    }

    desiredElevatorPosition = constants_StateMachine.TARGET_TO_PRESET_GROUP.get(desiredTargetState);

    // elevatorWasUp = s_Elevator.isSafeToMoveShooterAboveLimit();

    // if (elevatorWasUp) {
    //   subShooter.setDesiredPosition(desiredShooterPosition);
    // } else {
    //   s_Elevator.setElevatorPosition(desiredShooterPosition.elevatorPosition);
    // }

    // s_LEDs.clearAnimation();

    // switch (desiredTargetState) {
    //   case PREP_AMP:
    //     s_LEDs.setLEDs(constLEDs.PREP_AMP_COLOR);
    //     break;
    //   case PREP_SUB_BACKWARDS:
    //     s_LEDs.setLEDs(constLEDs.PREP_SUB_BACKWARDS_COLOR);
    //     break;
    //   case PREP_SPEAKER:
    //     s_LEDs.setLEDs(constLEDs.PREP_SPEAKER_COLOR);
    //     break;
    //   case PREP_NONE:
    //     if (!subTransfer.getGamePieceStored()) {
    //       s_LEDs.setLEDs(constLEDs.CLEAR_LEDS);
    //     } else {
    //       s_LEDs.setLEDAnimation(constLEDs.STORE_FEEDER_COLOR, 0);
    //     }
    //     break;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // s_Elevator.setElevatorPosition(desiredShooterPosition.elevatorPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (elevatorWasUp) {
    //   return subShooter.isShooterAtPosition(desiredShooterPosition.shooterAngle);
    // } else {
    //   return s_Elevator.isElevatorAtPosition(desiredShooterPosition.elevatorPosition);
    // }
    return true;
  }
}
