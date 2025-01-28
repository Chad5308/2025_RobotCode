package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.StateMachine.RobotState;

public class SourceState extends Command
{
    StateMachine s_StateMachine;
    Elevator s_Elevator;
    Vision s_Vision;
    Lights s_Lights;


    public SourceState(StateMachine s_StateMachine, Elevator s_Elevator, Vision s_Vision, Lights s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Elevator = s_Elevator;
        this.s_Vision = s_Vision;
        this.s_Lights = s_Lights;
    }



    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.SOURCE);
        //Create state for intaking from the source
    }
}
