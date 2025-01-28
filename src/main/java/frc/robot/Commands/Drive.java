package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.Constants.constants_OI;
import frc.robot.Util.DriverProfile;
import frc.robot.Util.Constants.constants_Drive;
import frc.robot.Subsystems.Drive.Swerve;


public class Drive extends Command{

    private final Swerve s_Swerve;
    public final DriverProfile driver;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
    public double ySpeed, xSpeed, turningSpeed;
    public ChassisSpeeds chassisSpeeds;

    



    // public DriveCommand(s_Swerve s_Swerve, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick) {
    public Drive(Swerve s_Swerve, DriverProfile driver) {

        this.s_Swerve = s_Swerve;
        this.xLimiter = new SlewRateLimiter(constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(constants_Drive.TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC);
        this.driver = driver;
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize()
    {
    //  s_Swerve.faceAllFoward();
    }

 


    @Override
    public void execute() {
        xSpeed = driver.holoX;
        ySpeed = driver.holoY;
        turningSpeed = driver.rotation;
        fieldOriented = s_Swerve.fieldOriented;

        xSpeed = Math.abs(xSpeed) > constants_OI.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > constants_OI.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > constants_OI.DEADBAND ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * constants_Drive.TELEDRIVE_MAX_SPEED_METERS_PER_SEC;
        ySpeed = yLimiter.calculate(ySpeed) * constants_Drive.TELEDRIVE_MAX_SPEED_METERS_PER_SEC;
        turningSpeed = turningLimiter.calculate(turningSpeed) * constants_Drive.TELEDRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC;

        drive();

        SmartDashboard.putNumber("Xspeed", xSpeed);
        SmartDashboard.putNumber("Yspeed", ySpeed);
        SmartDashboard.putNumber("Thetaspeed", turningSpeed);

    }
    
    public void drive()
    {
        if(fieldOriented)
        {
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
            chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, s_Swerve.getRotation2d());
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
