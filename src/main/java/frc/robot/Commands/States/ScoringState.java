package frc.robot.Commands.States;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;

public class ScoringState extends Command
{
    TargetState desiredTargetState;
    StateMachine s_StateMachine;
    Elevator s_Elevator;
    AlgaeRollers s_Rollers;
    Vision s_Vision;
    Lights s_Lights;
    Drive c_Drive;
    Climber s_Climber;

    public ScoringState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Elevator = s_Elevator;
        this.s_Rollers = s_Rollers;
        this.s_Vision = s_Vision;
        this.s_Lights = s_Lights;
        this.c_Drive = c_Drive;
        this.s_Climber = s_Climber;

        addRequirements(s_StateMachine);
    }


    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.SCORING);
        if(s_StateMachine.currentTargetState == TargetState.PREP_ALGAE)
        {
            //score algae!
        }else if(s_StateMachine.currentTargetState == TargetState.PREP_L1)
        {
            //score L1!
        }else if(s_StateMachine.currentTargetState == TargetState.PREP_L4)
        {
            //score l4!
        }else
        {
            //score l2 or l3!
        }
    }

    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        // If we don't have a game piece anymore, set the target state back to NONE
    //    if(s_Elevator.getGamePieceStored())
    //    {
    //         s_StateMachine.tryState(RobotState.CORAL, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
    //    }else if(s_Rollers.getGamePieceCollected())
    //    {
    //         s_StateMachine.tryState(RobotState.ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
    //    }else
    //    {
    //         s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
    //    }
        s_StateMachine.setTargetState(TargetState.PREP_NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
