package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase
{   
    public Spark blinken;
    // constructor method
    public Lights() 
    {
        blinken = new Spark(9);
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
