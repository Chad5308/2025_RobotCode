package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;

public class IntakingState extends Command 
{

    StateMachine s_StateMachine;
    Drive c_Drive;
    Elevator s_Elevator;
    Climber s_Climber;
    AlgaeRollers s_Rollers;
    Vision s_Vision;
    Lights s_Lights;

    frc.robot.Subsystems.StateMachine.RobotState previousState;

    public IntakingState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
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
      previousState = s_StateMachine.getRobotState();
        s_StateMachine.setRobotState(frc.robot.Subsystems.StateMachine.RobotState.INTAKE_ALGAE);
        //Bring down the intake and start the rollers
    }

    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if(previousState == frc.robot.Subsystems.StateMachine.RobotState.CORAL)
    {
      s_StateMachine.tryState(frc.robot.Subsystems.StateMachine.RobotState.COMBO, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
    }else
    {
      s_StateMachine.tryState(frc.robot.Subsystems.StateMachine.RobotState.ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Rollers.getGamePieceCollected();
  }
}
