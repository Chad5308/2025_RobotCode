package frc.robot.Util;

public class RobotMap
{
    public static final class MAP_CONTROLLER
    {
        public static final int MAIN_CONTROLLER_PORT = 0;
        public static final int OPP_CONTROLLER_PORT = 1;
        public static final int LEFT_JOYSTICK = 3;
        public static final int RIGHT_JOYSTICK = 4;
        //BUTTON BOARD
    }    

    public static final class MAP_DRIVETRAIN
    {
        public static final int NAVX_CAN = 0;

        //Front Left - Module 0
        public static final int FRONT_LEFT_DRIVE_CAN = 0;
        public static final int FRONT_LEFT_STEER_CAN = 0;
        public static final int FRONT_LEFT_ABS_ENCODER = 0;
        //Front Right - Module 1
        public static final int FRONT_RIGHT_DRIVE_CAN = 1;
        public static final int FRONT_RIGHT_STEER_CAN = 1;
        public static final int FRONT_RIGHT_ABS_ENCODER = 1;
        //Back Left - Module 2
        public static final int BACK_LEFT_DRIVE_CAN = 2;
        public static final int BACK_LEFT_STEER_CAN = 2;
        public static final int BACK_LEFT_ABS_ENCODER = 2;
        //Back Right - Module 3
        public static final int BACK_RIGHT_DRIVE_CAN = 3;
        public static final int BACK_RIGHT_STEER_CAN = 3;
        public static final int BACK_RIGHT_ABS_ENCODER = 3;
    }

    /*
    
    Elevator
        2 motors left& right
    Claw
        1 pitch motor
        1 Wheel motor
    Algae
        1 pitch motor
        1 wheel motor
    Climber
        1 climb motoor
        
    */
    public static final class MAP_ELEVATOR
    {
        public static final int ELEVATOR_LEFT = 10;
        public static final int ELEVATOR_RIGHT = 11;
    }
    public static final class MAP_CLAW
    {
        public static final int CLAW_PITCH = 20;
        public static final int CLAW_WHEEL = 21;
    }
    public static final class MAP_ALGAE
    {
        public static final int ALGAE_PITCH = 30;
        public static final int ALGAE_ROLLERS = 31;
    }
    public static final class MAP_CLIMBER
    {
        public static final int CLIMB = 40;
    }
    public static final class MAP_APRIL_TAGS
    {
        
    }
}   
