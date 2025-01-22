package frc.robot.Commands.States;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.TargetState;

public class IntakingState extends Command 
{

    StateMachine s_StateMachine;


    public IntakingState(StateMachine s_StateMachine)
    {
        this.s_StateMachine = s_StateMachine;

        addRequirements(s_StateMachine);
    }


    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(frc.robot.Subsystems.StateMachine.RobotState.INTAKE_ALGAE);
    }

    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
