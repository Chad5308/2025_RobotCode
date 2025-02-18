package frc.robot.Util;

//TODO Confirm that all these numbers have been set and are correct

public class RobotMap
{
    public static final class MAP_CONTROLLER
    {
        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;
        public static final int BUTTON_BOARD = 2;
        //BUTTON BOARD
    }    

    public static final class MAP_DRIVETRAIN
    {
        //Front Left - Module 0
        public static final int FRONT_LEFT_DRIVE_KRAKEN = 1;
        public static final int FRONT_LEFT_STEER_SPARKMAX = 1;
        public static final int FRONT_LEFT_ABS_ENCODER = 5;
        //Front Right - Module 1
        public static final int FRONT_RIGHT_DRIVE_KRAKEN = 2;
        public static final int FRONT_RIGHT_STEER_SPARKMAX = 2;
        public static final int FRONT_RIGHT_ABS_ENCODER = 6;
        //Back Right - Module 3
        public static final int BACK_RIGHT_DRIVE_KRAKEN = 3;
        public static final int BACK_RIGHT_STEER_SPARKMAX = 3;
        public static final int BACK_RIGHT_ABS_ENCODER = 7;
        //Back Left - Module 2
        public static final int BACK_LEFT_DRIVE_KRAKEN = 4;
        public static final int BACK_LEFT_STEER_SPARKMAX = 4;
        public static final int BACK_LEFT_ABS_ENCODER = 8;
    }
    public static final class MAP_ELEVATOR
    {
        public static final int ELEVATOR_LEFT_SPARKMAX = 10;
        public static final int ELEVATOR_RIGHT_SPARKMAX = 11;
        public static final int ELEVATOR_ROLLERS_SPARKMAX = 12;
        public static final int ELEVATOR_CANRANGE = 13;
        public static final int ELEVATOR_ABS_ENCODER_PWM_PORT = 1; //TODO Check this
    }
    public static final class MAP_ALGAE
    {
        public static final int ALGAE_PITCH = 20;
        public static final int ALGAE_ROLLERS = 21;
        public static final int ALGAE_CANRANGE = 22;
        public static final int ALGAE_ABS_ENCODER_PWM_PORT = 2; //TODO Check this too
    }
    public static final class MAP_CLIMBER
    {
        public static final int CLIMB_SPARKMAX = 30;
        public static final int CLIMB_ABS_ENCODER_PWM_PORT = 3; //TODO Check this aswell
    }
    public static final class MAP_APRIL_TAGS //[id num, height in inches, coordinate x, coordinate y, heading] inches to meters; use field manual image for id reference
    {
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
    {   // coral = blue; algae = green; color 2 = blue; color 1 = green
        // MAKE PREP STATES A PATTERN OF THEIR PARENT STATE (IF PWM_NONE IS GRAY, MAKE PWM_PREP_NONE FLASHING GRAY OR SOMETHING)
        public static final double PWM_NONE_COLOR = 0.93; // SOLID WHITE
        public static final double PWM_SOURCE_COLOR = 0.33; // COLOR 2 SHOT
        public static final double PWM_INTAKE_ALGAE_PATTERN = 0.15; // COLOR 1 STROBE
        public static final double PWM_CORAL_COLOR = 0.81; // AQUA
        public static final double PWM_ALGAE_COLOR = 0.75; // GREEN
        public static final double PWM_COMBO_COLOR = 0.67; // GOLD
        public static final double PWM_PREP_L1_PATTERN = 0.35; // COLOR 2 STROBE
        public static final double PWM_PREP_L2_PATTERN = 0.35; // COLOR 2 STROBE
        public static final double PWM_PREP_L3_PATTERN = 0.35; // COLOR 2 STROBE
        public static final double PWM_PREP_NONE_PATTERN = -0.05; // STROBE WHITE
        public static final double PWM_PREP_ALGAE_PATTERN = 0.13; // COLOR 1 shot
        public static final double PWM_CLEAN_L2_PATTERN = -0.11; // red strobe
        public static final double PWM_CLEAN_L3_PATTERN = -0.11; // red strobe
        public static final double PWM_CLIMBING_COLOR = 0.57; // hot pink 
        public static final double PWM_SCORING = -0.07; // gold strobe
    }
}   
