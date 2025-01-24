package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.StateMachine.RobotState;

public class CoralState extends Command
{
    StateMachine s_StateMachine;
    Elevator s_Elevator;
    Lights s_Lights;
    Vision s_Vision;
    boolean combo;


    public CoralState(StateMachine s_StateMachine, Elevator s_Elevator, Vision s_Vision, Lights s_Lights, boolean combo)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Elevator = s_Elevator;
        this.s_Lights = s_Lights;
        this.s_Vision = s_Vision;
        this.combo = combo;

        addRequirements(s_StateMachine);
    }

    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.CORAL);
        //Sequence for picking up coral but also check first if there is already a game piece held and if so that means we came from Score -> None -> Coral
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        //not gonna end unless taken out by combo
        return combo;
    }

}
