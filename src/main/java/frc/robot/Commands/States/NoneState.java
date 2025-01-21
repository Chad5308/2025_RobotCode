package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.Vision;

public class NoneState extends Command 
{
    
    public NoneState(RobotState desiredState, StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator,
     Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights) 
    {

    }
}
