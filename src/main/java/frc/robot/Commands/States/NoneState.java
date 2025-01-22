package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Subsystems.Vision;

public class NoneState extends Command 
{
    StateMachine s_StateMachine;
    
    public NoneState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator,
     Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights) 
    {
        this.s_StateMachine = s_StateMachine;

        addRequirements(s_StateMachine);
    }


    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.NONE);
        s_StateMachine.setTargetState(TargetState.PREP_NONE);
    }


    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    //Set Stuff to home positions
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
