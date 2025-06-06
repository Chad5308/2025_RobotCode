
package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Util.Constants.constants_Elevator;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;
import frc.robot.Subsystems.Vision;

public class CleanL3State extends Command
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
        s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_CLEAN_L3_PATTERN);
        s_Elevator.setElevatorPosition(constants_Elevator.CLEAN_L3);
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