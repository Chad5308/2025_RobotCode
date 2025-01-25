package frc.robot.Util;

public class RobotMap
{
    public static final class MAP_CONTROLLER
    {
        public static final int MAIN_CONTROLLER_PORT = 0;
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
    public static final class MAP_PWM_LIGHTS
    {   // 
        // MAKE PREP STATES A PATTERN OF THEIR PARENT STATE (IF PWM_NONE IS GRAY, MAKE PWM_PREP_NONE FLASHING GRAY OR SOMETHING)
        public static final double PWM_NONE_COLOR = 0;
        public static final double PWM_SOURCE_COLOR = 0;
        public static final double PWM_INTAKE_ALGAE_COLOR = 0;
        public static final double PWM_CORAL_COLOR = 0.85; // DARK BLUE
        public static final double PWM_ALGAE_COLOR = 0.75; // DARK GREEN
        public static final double PWM_COMBO_COLOR = 0.79; // BLUE GREEN
        public static final double PWM_PREP_L1_PATTERN = 0.59;
        public static final double PWM_PREP_L2_PATTERN = 0.69;
        public static final double PWM_PREP_L3_PATTERN = 0.33;
        public static final double PWM_PREP_L4_PATTERN = 0.21;
        public static final double PWM_PREP_NONE_PATTERN = 0;// NONE COLOR LARSON SCANNER
        public static final double PWM_PREP_ALGAE_PATTERN = 0;// GREEN LARSON SCANNER
        public static final double PWM_CLEAN_L2_PATTERN = 0;
        public static final double PWM_CLEAN_L3_PATTERN = 0;
        public static final double PWM_CLIMBING_COLOR = 0;

        

    }
}   
