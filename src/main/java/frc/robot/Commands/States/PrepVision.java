package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;

public class PrepVision extends Command
{
    StateMachine s_StateMachine;
    Drive c_Drive;
    Lights s_Lights;
    

    public PrepVision(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Lights s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.c_Drive = c_Drive;
        this.s_Lights = s_Lights;
    }
}
