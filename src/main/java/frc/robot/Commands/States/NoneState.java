package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Util.Constants.constants_Elevator;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;
import frc.robot.Subsystems.Vision;

public class NoneState extends Command 
{
    StateMachine s_StateMachine;
    Drive c_Drive;
    Elevator s_Elevator;
    Climber s_Climber;
    AlgaeRollers s_Rollers;
    Vision s_Vision;
    Lights s_Lights;
     
    Trigger gamePieceStoredTrigger_Coral = new Trigger(() -> s_Elevator.getGamePieceStored());
    Trigger gamePieceCollectedTrigger_Algae = new Trigger(() -> s_Rollers.getGamePieceCollected());

    
    public NoneState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights) 
    {
        this.s_StateMachine = s_StateMachine;
        this.c_Drive = c_Drive;
        this.s_Elevator = s_Elevator;
        this.s_Climber = s_Climber;
        this.s_Rollers = s_Rollers;
        this.s_Vision = s_Vision;
        this.s_Lights = s_Lights;

        addRequirements(s_StateMachine);
    }


    @Override
    public void initialize()
    {
      s_StateMachine.setRobotState(RobotState.NONE);
      s_StateMachine.setTargetState(TargetState.PREP_NONE);
      s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_NONE_COLOR);
      //run our homeing sequence
      s_Elevator.setElevatorPosition(constants_Elevator.CORAL);
      s_Rollers.retractIntake();
    }
    
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
