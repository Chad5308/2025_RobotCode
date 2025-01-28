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
         //[id num, height in inches, coordinate x, coordinate y, heading] inches to meters; use field manual image for id reference
    public static final double[] RED_CS_X = {1, 1.4859, 16.6992, 0.6553, 126};
    public static final double[] RED_CS_Y = {2, 1.4859, 16.6992, 7.4048, 234};
    public static final double[] RED_PROCESSOR = {3, 1.3017, 11.6087, 8.0641, 270};
    public static final double[] RED_SIDE_BLUE_BARGE = {4, 1.8678, 9.2800, 6.1340, 0};
    public static final double[] RED_SIDE_RED_BARGE = {5, 1.8678, 9.2800, 1.9148, 0};
    public static final double[] RED_REEF_KL = {6, 0.3081, 13.4742, 3.3074, 300};
    public static final double[] RED_REEF_AB = {7, 0.3081, 13.8934, 4.0262, 0};
    public static final double[] RED_REEF_CD = {8, 0.3081, 13.4742, 4.7449, 60};
    public static final double[] RED_REEF_EF = {9, 0.3081, 12.6464, 4.7449, 120};
    public static final double[] RED_REEF_GH = {10, 0.3081, 12.2194, 4.0262, 180};
    public static final double[] RED_REEF_IJ = {11, 0.3081, 12.6464, 3.3074, 240};
    public static final double[] BLUE_CS_Y = {12, 1.4859, 0.8507, 0.6553, 54};
    public static final double[] BLUE_CS_X = {13, 1.4859, 0.8507, 7.4048, 306};
    public static final double[] BLUE_PROCESSOR = {14, 1.8678, 8.2744, 6.1340, 0};
    public static final double[] BLUE_SIDE_BLUE_BARGE = {15, 1.8678, 8.2744, 1.9148, 180};
    public static final double[] BLUE_SIDE_RED_BARGE = {16, 1.3017, 5.9915, -0.0004, 90};
    public static final double[] BLUE_REEF_CD = {17, 0.3081, 4.0734, 3.3074, 240};
    public static final double[] BLUE_REEF_AB = {18, 0.3081, 3.6571, 4.0262, 0};
    public static final double[] BLUE_REEF_KL = {19, 0.3081, 4.0734, 4.7449, 120};
    public static final double[] BLUE_REEF_IJ = {20, 0.3081, 4.9068, 4.7449, 180};
    public static final double[] BLUE_REEF_GH = {21, 0.3081, 5.3246, 5.3246, 0};
    public static final double[] BLUE_REEF_EF = {22, 0.3081, 4.9068, 3.3074, 300};
        
    }
    public static final class MAP_PWM_LIGHTS
    {   // 
        // MAKE PREP STATES A PATTERN OF THEIR PARENT STATE (IF PWM_NONE IS GRAY, MAKE PWM_PREP_NONE FLASHING GRAY OR SOMETHING)
        public static final double PWM_NONE_COLOR = 0; 
        public static final double PWM_CS_COLOR = 0;
        public static final double PWM_INTAKE_ALGAE_COLOR = 0;
        public static final double PWM_CORAL_COLOR = -0.01; // COLOR 1 LARSON SCANNER
        public static final double PWM_ALGAE_COLOR = 0.19; // COLOR 2 LARSON SCANNER
        public static final double PWM_COMBO_COLOR = 0.41; // COLOR 1 AND 2 COLOR GRADIENT
        public static final double PWM_PREP_L1_PATTERN = 0.59;
        public static final double PWM_PREP_L2_PATTERN = 0.69;
        public static final double PWM_PREP_L3_PATTERN = 0.33;
        public static final double PWM_PREP_L4_PATTERN = 0.21;
        public static final double PWM_PREP_NONE_PATTERN = 0;
        public static final double PWM_PREP_ALGAE_PATTERN = 0;
        public static final double PWM_CLEAN_L2_PATTERN = 0;
        public static final double PWM_CLEAN_L3_PATTERN = 0;
        public static final double PWM_CLIMBING_COLOR = 0.93; // SOLID WHITE

        

    }
}   
