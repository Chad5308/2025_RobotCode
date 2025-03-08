package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Util.Constants.constants_Elevator;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;

public class CoralState extends Command
{
    StateMachine s_StateMachine;
    Elevator s_Elevator;
    Lights s_Lights;
    Vision s_Vision;
    AlgaeRollers s_Rollers;


    public CoralState(StateMachine s_StateMachine, Elevator s_Elevator, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Elevator = s_Elevator;
        this.s_Lights = s_Lights;
        this.s_Rollers = s_Rollers;
        this.s_Vision = s_Vision;

        addRequirements(s_StateMachine);
    }

    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.CORAL);
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_CORAL_COLOR);
        s_Elevator.setElevatorPosition(constants_Elevator.CORAL); 
        if(s_Rollers.getGamePieceCollected())
        {
            s_Rollers.ROLLERS.set(0);
        }else
        {
            s_Rollers.retractIntake.schedule();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return true;
    }

}
