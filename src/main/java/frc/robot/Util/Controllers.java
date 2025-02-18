package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;

public class Controllers
{
    public CommandXboxController xboxController;
    public CommandJoystick leftStick, rightStick;
    public Trigger PREP_L1, PREP_L2, PREP_L3, PREP_ALGAE, PREP_NONE, CLIMB_UP, CLIMB_DOWN, CLIMB_AUTO, ELE_UP, ELE_DOWN, ELE_ROLLER;
    public CommandGenericHID buttonBoard;
    public String driveControls;
    public boolean rightStickDrive;
    
    public Trigger a, b, x, y, rb, lb, rt, lt, rs, ls, square, lines, povL, povR, povUp, povDown;

    public Controllers(String driveControls, boolean rightStickDrive)
    {
        this.driveControls = driveControls;
        this.rightStickDrive = rightStickDrive;
        buttonBoard = new CommandGenericHID(MAP_CONTROLLER.BUTTON_BOARD);
        initialize_Controls();
        
        if(driveControls == "Joysticks")
        {
            leftStick = new CommandJoystick(MAP_CONTROLLER.LEFT_JOYSTICK);
            rightStick = new CommandJoystick(MAP_CONTROLLER.RIGHT_JOYSTICK);
        }else
        {
            xboxController = new CommandXboxController(MAP_CONTROLLER.XBOX_CONTROLLER);
        }
    }

    public double getDriveXAxis()
    {
        switch (driveControls)
        {
            case "Joysticks":
                return rightStickDrive? rightStick.getX(): leftStick.getX();
            case"Xbox":
                return rightStickDrive? xboxController.getRightX(): xboxController.getLeftX();
        }
    
        return 0;
    }

     public double getDriveYAxis()
    {
        switch (driveControls)
        {
            case "Joysticks":
                return rightStickDrive? rightStick.getY(): leftStick.getY();
            case"Xbox":
                return rightStickDrive? xboxController.getRightY(): xboxController.getLeftY();
        }
        return 0;
    }

    public double getSteerXAxis()
    {
        switch (driveControls)
        {
            case "Joysticks":
                return rightStickDrive? leftStick.getX(): rightStick.getX();
            case"Xbox":
                return rightStickDrive? xboxController.getLeftX(): xboxController.getRightX();
        }
        return 0;
    }

    public Trigger getButton(String Name)
    {
        switch (Name)
        {
            case "Algae":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? leftStick.trigger(): rightStick.trigger();
                    case "Xbox":
                        return xboxController.leftTrigger();
                }
                break;
            case "Source":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? leftStick.button(2): rightStick.button(2);
                    case "Xbox":
                        return xboxController.leftBumper();
                }
                break;
            case "Clean L2":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? leftStick.button(3): rightStick.button(3);
                    case "Xbox":
                        return xboxController.a();
                }
                break;
            case "Clean L3":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? leftStick.button(4): rightStick.button(4);
                    case "Xbox":
                        return xboxController.b();
                }
                break;
            case "Score":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? rightStick.trigger():leftStick.trigger();
                    case "Xbox":
                        return xboxController.rightTrigger();
                }
                break;
            case "Reset Heading":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? rightStick.button(2): leftStick.button(2);
                    case "Xbox":
                        return xboxController.povRight();
                }
                break;
            case "Toggle FO":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? rightStick.button(3): leftStick.button(3);
                    case "Xbox":
                        return xboxController.povLeft();
                }
                break;
            case "Reset Wheels":
                switch (driveControls)
                {
                    case "Joysticks":
                        return rightStickDrive? rightStick.button(4): leftStick.button(4);
                    case "Xbox":
                        return xboxController.button(7);
                }
        }
        Commands.print("Invalid Control Requested");
        return new Trigger(null);
    }


    public void initialize_Controls()
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
}
