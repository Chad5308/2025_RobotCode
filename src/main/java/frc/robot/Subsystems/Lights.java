package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RobotMap.MAP_PWM_LIGHTS;

public class Lights extends SubsystemBase
{   
    public Spark blinken;
    // constructor method
    public Lights() 
    {
        // blinken1 = new Spark(MAP_PWM_LIGHTS.BLINKEN_1_PORT);
        blinken = new Spark(MAP_PWM_LIGHTS.BLINKEN_PORT);
        // Spark is a class so blinken is a new instance of it
    }
    // get method
    public double getNumber() 
    {
        return blinken.get();
    }
    // set method
    public void setNumber(double PWM_NUMBER)
    {
        blinken.set(PWM_NUMBER);
    }

}
