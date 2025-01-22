package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.Constants.constants_OI;
import frc.robot.Util.Constants.constants_Drive;
import frc.robot.Subsystems.Drive.Swerve;


public class Drive extends Command{
    

    
    private final Swerve s_Swerve;
    public final CommandXboxController opController;
    // public final CommandJoystick leftStick;
    // public final CommandJoystick rightStick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed, xSpeed, turningSpeed;
     public double ll_zSpeed, ll_xSpeed, ll_turningSpeed;
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);

    



    // public DriveCommand(s_Swerve s_Swerve, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick) {
        public Drive(Swerve s_Swerve, CommandXboxController opController) {

                this.s_Swerve = s_Swerve;
                this.xLimiter = new SlewRateLimiter(constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC);
                this.yLimiter = new SlewRateLimiter(constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC);
                this.turningLimiter = new SlewRateLimiter(constants_Drive.TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC);
                addRequirements(s_Swerve);
                this.opController = opController;
                // this.leftStick = leftStick;
                // this.rightStick = rightStick;
    }


    @Override
    public void initialize()
    {
    //  s_Swerve.faceAllFoward();
    }

 


    @Override
    public void execute() {
      
        // xSpeed = IsJoyStick? -leftStick.getX(): -opController.getLeftX();
        // ySpeed = IsJoyStick? -leftStick.getY(): -opController.getLeftY();
        // turningSpeed = IsJoyStick? -rightStick.getX(): -opController.getRightX();
        xSpeed = -opController.getLeftX();
        ySpeed = -opController.getLeftY();
        turningSpeed = -opController.getRightX();
        fieldOriented = s_Swerve.fieldOriented;


        
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);


        xSpeed = Math.abs(xSpeed) > constants_OI.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > constants_OI.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > constants_OI.DEADBAND ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * constants_Drive.TELEDRIVE_MAX_SPEED_METERS_PER_SEC;
        ySpeed = yLimiter.calculate(ySpeed) * constants_Drive.TELEDRIVE_MAX_SPEED_METERS_PER_SEC;
        turningSpeed = turningLimiter.calculate(turningSpeed) * constants_Drive.TELEDRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC;

        
        drive();
    }
    
    public void drive()
    {
        if(fieldOriented)
        {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, s_Swerve.getRotation2d());
        }else
        {
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
        }
        s_Swerve.setModuleStates(chassisSpeeds);        
    }


    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
