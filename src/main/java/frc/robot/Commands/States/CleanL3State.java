package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;
import frc.robot.Subsystems.Vision;


 // this is nathan, I'm making this state from what I know and looking
 //  at the other already made states. for the packages, I don't know which
 //  ones are necessary and which ones aren't. If you know what's redundant
 //  what's not, please go ahead and change it
public class CleanL3State extends Command //TODO
{
    StateMachine s_StateMachine;
    Drive c_Drive;
    Elevator s_Elevator;
    Climber s_Climber;
    AlgaeRollers s_Rollers;
    Lights s_Lights;
    Vision s_Vision;

        public CleanL3State(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.c_Drive = c_Drive;
        this.s_Elevator = s_Elevator;
        this.s_Climber = s_Climber;
        this.s_Rollers = s_Rollers;
        this.s_Lights = s_Lights;
        this.s_Vision = s_Vision;

        addRequirements(s_StateMachine);
    }
    // set up
    @Override
    public void initialize()
    {
            s_StateMachine.setRobotState(RobotState.CLEAN_L3);
            s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_CORAL_COLOR);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // loop
    @Override
    public void execute()
    {

    }
     // Called once the command ends or is interrupted.
     public void end(boolean interrupted)
     {

     }

     // Returns true whe nthe comand should end.
     @Override
     public boolean isFInished()
        {
         return combo;
        }
     
}