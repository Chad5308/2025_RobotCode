package frc.robot.Commands.States;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Util.Constants.*;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;
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
        if(s_Rollers.getGamePieceCollected() && s_Elevator.getGamePieceStored()) //What to do when we are now picking up a Coral
        { 
            s_Elevator.setElevatorPosition(constants_Elevator.CORAL); 
            s_Rollers.setAlgaeIntake(constants_Rollers.ALGAE);
            s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_COMBO_COLOR);
            s_StateMachine.setRobotState(RobotState.COMBO);
        }
         else if (s_Rollers.getGamePieceCollected())
        {
            s_Rollers.setAlgaeIntake(constants_Rollers.ALGAE);
            s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_ALGAE_COLOR);
            s_StateMachine.setRobotState(RobotState.ALGAE);

        } 
         else if (s_Elevator.getGamePieceStored())
        {
            s_Elevator.setElevatorPosition(constants_Elevator.CORAL);
            s_Lights.setNumber(MAP_PWM_LIGHTS.PWM_CORAL_COLOR);
            s_StateMachine.setRobotState(RobotState.CORAL);
            
        }
        
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished() {
        return false;
    }  
}
