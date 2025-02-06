package frc.robot.Commands.States;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Util.Constants.constants_Elevator;
import frc.robot.Util.Constants.constants_Rollers;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;

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
    public void execute()
    {
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_SCORING);

        if(s_StateMachine.getRobotState() == RobotState.PREP_ALGAE)
        {
            s_Rollers.setAlgaeIntake(constants_Rollers.SCORING);
            s_StateMachine.setRobotState(RobotState.SCORING);
        }
        else
        {
            s_Elevator.setElevatorPosition(constants_Elevator.SCORE);
            s_StateMachine.setRobotState(RobotState.SCORING);
        }        
    }

    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        if((!s_Rollers.getGamePieceCollected() && s_Elevator.getGamePieceStored()))
        {
            s_StateMachine.setTargetState(TargetState.PREP_NONE);
            s_Elevator.setElevatorPosition(constants_Elevator.PREP_NONE);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
