package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.Trigger;



public class DriverProfile
{
    public String name;
    public Trigger intakeAlgae, intakeCoral, cleanL2, cleanL3, PREP_L1, PREP_L2, PREP_L3, PREP_L4, PREP_NONE, PREP_ALGAE, score, zeroHeading, FOToggle, resetWheels; //add climber controls
    public double holoX, holoY, rotation;
    public ControllerTypes currentController;


    public static enum ControllerTypes
    {
        Xbox, ps4, ps5, JoySticks
    }
    
    public DriverProfile(ControllerTypes driverController, ControllerTypes oppController, boolean isDriver, String name, Trigger intakeAlgae, Trigger intakeCoral, Trigger cleanL2, Trigger cleanL3, Trigger score, Trigger zeroHeading, Trigger FOToggle, Trigger resetWheels, double holoX, double holoY, double rotation, Trigger PREP_L1, Trigger PREP_L2, Trigger PREP_L3, Trigger PREP_L4, Trigger PREP_ALGAE, Trigger PREP_NONE)
    {
       currentController = isDriver ? driverController : oppController;
       this.name = name;
       setDriveControlls(intakeAlgae, intakeCoral, cleanL2, cleanL3, score, zeroHeading, FOToggle, resetWheels, holoX, holoY, rotation);
       setOppControlls(PREP_L1, PREP_L2, PREP_L3, PREP_L4, PREP_ALGAE, PREP_NONE);
    }

    public void setDriveControlls(Trigger intakeAlgae, Trigger intakeCoral, Trigger cleanL2, Trigger cleanL3, Trigger score, Trigger zeroHeading, Trigger FOToggle, Trigger resetWheels, double holoX, double holoY, double rotation)
    {
        this.intakeAlgae = intakeAlgae;
        this.intakeCoral = intakeCoral;
        this.cleanL2 = cleanL2;
        this.cleanL3 = cleanL3;
        this.score = score;
        this.zeroHeading = zeroHeading;
        this.FOToggle = FOToggle;
        this.resetWheels = resetWheels;
        this.holoX = holoX;
        this.holoY = holoY;
        this.rotation = rotation;
    }
    public void setOppControlls(Trigger PREP_L1, Trigger PREP_L2, Trigger PREP_L3, Trigger PREP_L4, Trigger PREP_ALGAE, Trigger PREP_NONE)
    {
        this.PREP_L1 = PREP_L1;
        this.PREP_L2 = PREP_L2;
        this.PREP_L3 = PREP_L3;
        this.PREP_L4 = PREP_L4;
        this.PREP_ALGAE = PREP_ALGAE;
        this.PREP_NONE = PREP_NONE;
    }

    public ControllerTypes currentController()
    {
        return currentController;
    }
      
    
}




