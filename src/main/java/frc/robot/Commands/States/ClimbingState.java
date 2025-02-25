package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Util.Constants.constants_Climber;
import frc.robot.Util.RobotMap.MAP_CLIMBER;
import frc.robot.Subsystems.Vision;

public class ClimbingState extends Command 
{
    StateMachine s_StateMachine;
    Elevator s_Elevator;
    Climber s_Climber;
    Lights s_Lights;
    Vision s_Vision;


    public ClimbingState(StateMachine s_StateMachine, Elevator s_Elevator, Climber s_Climber, Vision s_Vision, Lights s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Elevator = s_Elevator;
        this.s_Climber = s_Climber;
        this.s_Lights = s_Lights;
        this.s_Vision = s_Vision;

        addRequirements(s_StateMachine);
    }

    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.CLIMBING);
        //Climbing sequence
        s_Climber.setClimber(constants_Climber.ACTIVE);
    }
    
    @Override
    public void execute()
    {

    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
