package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Subsystems.Vision;

public class ComboState extends Command
{
    StateMachine s_StateMachine;
    Drive c_Drive;
    Elevator s_Elevator;
    Climber s_Climber;
    AlgaeRollers s_Rollers;
    Lights s_Lights;
    Vision s_Vision;


    public ComboState(StateMachine s_StateMachine, Drive c_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
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


    @Override
    public void initialize()
    {
        if(s_StateMachine.currentState == RobotState.ALGAE)
        {
            //What to do when we are now picking up a coral
            new CoralState(s_StateMachine, s_Elevator, s_Vision, s_Lights, true);
        }else if(s_StateMachine.currentState == RobotState.INTAKE_ALGAE)
        {
            //what to do when we are now picking up a coral
            new AlgaeState(s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights, true);
        }

        s_StateMachine.setRobotState(RobotState.COMBO);
        s_StateMachine.setTargetState(TargetState.PREP_ALGAE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        //not gonna end unless taken out by combo
    }

      // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }  
}
