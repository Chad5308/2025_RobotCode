package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.StateMachine.RobotState;

public class AlgaeState extends Command
{
    StateMachine s_StateMachine;
    Elevator s_Elevator;
    AlgaeRollers s_Rollers;
    Lights s_Lights;
    Vision s_Vision;
    boolean combo;
    Drive c_Drive;
    Climber s_Climber;


    public AlgaeState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights, boolean combo)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Elevator = s_Elevator;
        this.s_Rollers = s_Rollers;
        this.s_Lights = s_Lights;
        this.s_Vision = s_Vision;
        this.combo = combo;
        this.c_Drive = c_Drive;
        this.s_Climber = s_Climber;



        addRequirements(s_StateMachine);
    }


    @Override
    public void initialize()
    {
        s_StateMachine.setRobotState(RobotState.ALGAE);

        //Write Command to retract intake and stop rollers
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return combo;
        //not gonna end unless taken out by combo
    }
}
