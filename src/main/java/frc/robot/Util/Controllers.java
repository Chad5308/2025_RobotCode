package frc.robot.Util;


import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;

public class Controllers
{
    public CommandJoystick leftStick, rightStick;
    public Trigger PREP_L1, PREP_L2, PREP_L3, PREP_ALGAE, PREP_NONE, CLIMB_UP, CLIMB_DOWN, CLIMB_AUTO, ELE_UP, ELE_DOWN, ELE_ROLLER, CORAL_OVERRIDE, ALGAE_OVERRIDE;
    public CommandGenericHID buttonBoard;
    public CommandXboxController xbox;
    

    public Controllers()
    {
        leftStick = new CommandJoystick(MAP_CONTROLLER.LEFT_JOYSTICK);
        rightStick = new CommandJoystick(MAP_CONTROLLER.RIGHT_JOYSTICK);

        xbox = new CommandXboxController(MAP_CONTROLLER.XBOX_CONTROLLER);
        initialize_Xbox_Controls();

        // buttonBoard = new CommandGenericHID(MAP_CONTROLLER.BUTTON_BOARD);
        // initialize_ButtonBoard_Controls();
    }

    public void initialize_ButtonBoard_Controls()
    {
        PREP_L1 = buttonBoard.button(1);
        PREP_L2 = buttonBoard.button(2);
        PREP_L3 = buttonBoard.button(3);
        PREP_ALGAE = buttonBoard.button(4);
        PREP_NONE = buttonBoard.button(6);
        CLIMB_UP = buttonBoard.button(7);
        CLIMB_DOWN = buttonBoard.button(8);
        CLIMB_AUTO = buttonBoard.button(9);
        ELE_UP = buttonBoard.button(10);
        ELE_DOWN = buttonBoard.button(11);
        ELE_ROLLER = buttonBoard.button(12);
    
    }
    public void initialize_Xbox_Controls()
    {
        PREP_L1 = xbox.leftTrigger();
        PREP_L2 = xbox.leftBumper();
        PREP_L3 = xbox.rightTrigger();
        PREP_ALGAE = xbox.rightBumper();
        PREP_NONE = xbox.b();
        CLIMB_UP = xbox.povUp();
        CLIMB_DOWN = xbox.povDown();
        ELE_UP = xbox.y();
        ELE_DOWN = xbox.a();
        ELE_ROLLER = xbox.x();
        CORAL_OVERRIDE = xbox.povLeft();
        ALGAE_OVERRIDE = xbox.povRight();

    }

}
