package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;

public class Lights extends SubsystemBase
{   
    public Spark blinken1, blinken2;
    // constructor method
    public Lights() 
    {
        blinken1 = new Spark(MAP_PWM_LIGHTS.BLINKEN_1_PORT);
        blinken2 = new Spark(MAP_PWM_LIGHTS.BLINKEN_2_PORT);
        // Spark is a class so blinken is a new instance of it
    }
    // get method
    public double getNumber() 
    {
        return blinken1.get();
    }
    // set method
    public void setNumber(double PWM_NUMBER)
    {
        blinken1.set(PWM_NUMBER);
    }

}
