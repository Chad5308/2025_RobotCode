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

// getting and setting example
// public double getPosition()
//     {
//         return PITCH_ENCODER.getPosition() * 360;
//     }

//     public double getSpeed()
//     {
//         return ROLLERS_ENCODER.getVelocity();
//     }
        
//     public boolean getGamePieceCollected()
//     {
//         return testBool;
//         //TODO Code this distance/proxy sensor in and return senser reading
//     }

//     public void setPosition(double position) //degrees
//     {
//         PITCH_PID.setReference(position, ControlType.kPosition);
//     }

//     public void setSpeed(double speed) //rps
//     {
//         ROLLERS_PID.setReference(speed, ControlType.kVelocity);
//     }