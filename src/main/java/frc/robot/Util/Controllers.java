package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface Controllers
{


    public class Xbox implements Controllers
    {
        public CommandXboxController xbox;
        public Xbox(int port)
        {
            xbox = new CommandXboxController(port);
        }

        public CommandXboxController getController()
        {
            return xbox;
        }
    }

    public static class PS4 implements Controllers
    {
        public CommandPS4Controller ps4;
        public PS4(int port)
        {
            ps4 = new CommandPS4Controller(port);
        }
        public CommandPS4Controller getController()
        {
            return ps4;
        }
    }

    public static class JoySticks implements Controllers
    {
        public CommandJoystick left, right;
        public JoySticks(int L, int R)
        {
            left = new CommandJoystick(L);
            right = new CommandJoystick(R);
        }
        public CommandJoystick getLeftController()
        {
            return left;
        }
        public CommandJoystick getRightController()
        {
            return right;
        }
    }

    public static class PS5 implements Controllers
    {
        public CommandPS5Controller ps5;
        public PS5(int port)
        {
            ps5 = new CommandPS5Controller(port);
        }
        public CommandPS5Controller getController()
        {
            return ps5;
        }
    }
}

