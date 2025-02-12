package frc.robot.Subsystems.Drive;

import java.util.Optional;

import com.studica.frc.AHRS;

// import com.studica.frc.AHRS;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.Constants.constants_Drive;
import frc.robot.Util.Constants.constants_OI;
import frc.robot.Util.RobotMap.MAP_DRIVETRAIN;

public class Swerve extends SubsystemBase{
    public final CommandXboxController opController = new CommandXboxController(constants_OI.OP_CONTROLLER_PORT);
    public boolean fieldOriented = false;
    public boolean hasReset = false;
    public boolean isRedAlliance;
    Optional<Alliance> alliance;
  
    
    public static Module frontLeftModule = new Module(MAP_DRIVETRAIN.FRONT_LEFT_STEER_CAN, MAP_DRIVETRAIN.FRONT_LEFT_DRIVE_CAN, constants_Drive.FL_DRIVE_ENCODER_REVERSED, constants_Drive.FL_STEER_ENCODER_REVERSED, MAP_DRIVETRAIN.FRONT_LEFT_ABS_ENCODER, constants_Drive.FL_OFFSET, constants_Drive.FL_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    public static Module frontRightModule = new Module(MAP_DRIVETRAIN.FRONT_RIGHT_STEER_CAN, MAP_DRIVETRAIN.FRONT_RIGHT_DRIVE_CAN, constants_Drive.FR_DRIVE_ENCODER_REVERSED, constants_Drive.FR_STEER_ENCODER_REVERSED, MAP_DRIVETRAIN.FRONT_RIGHT_ABS_ENCODER, constants_Drive.FR_OFFSET, constants_Drive.FR_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    public static Module backLeftModule = new Module(MAP_DRIVETRAIN.BACK_LEFT_STEER_CAN, MAP_DRIVETRAIN.BACK_LEFT_DRIVE_CAN, constants_Drive.BL_DRIVE_ENCODER_REVERSED, constants_Drive.BL_STEER_ENCODER_REVERSED, MAP_DRIVETRAIN.BACK_LEFT_ABS_ENCODER, constants_Drive.BL_OFFSET, constants_Drive.BL_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    public static Module backRightModule = new Module(MAP_DRIVETRAIN.BACK_RIGHT_STEER_CAN, MAP_DRIVETRAIN.BACK_RIGHT_DRIVE_CAN, constants_Drive.BR_DRIVE_ENCODER_REVERSED, constants_Drive.BR_STEER_ENCODER_REVERSED, MAP_DRIVETRAIN.BACK_RIGHT_ABS_ENCODER, constants_Drive.BR_OFFSET, constants_Drive.BR_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    
    
    public Swerve() 
    {
        new Thread(() -> {try
        {
            Thread.sleep(500);
            zeroHeading();
        } catch (Exception e) {}}).start();    
        alliance = getAlliance();
    }
        
    //gyro int and heading code
    private AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
    public void zeroHeading() 
    {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }
    
    public void setAngleAdjustment(double offset)
    {
        gyro.setAngleAdjustment(offset);
    }
    
    public double getHeading() 
    {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
    
    public Rotation2d getRotation2d() 
    {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    public boolean allianceCheck() 
    {
        if (alliance.isPresent() && (alliance.get() == Alliance.Red)) {isRedAlliance = true;}else{isRedAlliance = false;}
        return isRedAlliance;
    }
    
    public Optional<Alliance> getAlliance() 
    {
        return DriverStation.getAlliance();
    }
    
    //Odometer code
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry
    (
        constants_Drive.kDriveKinematics,
        getRotation2d(),
        getModulePositions()
    );
    
    public Pose2d getPose() 
    {
        return odometer.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) 
    {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
    
    public static SwerveModulePosition[] getModulePositions()
    {
        return new SwerveModulePosition[]{
            frontLeftModule.getModulePosition(),
            frontRightModule.getModulePosition(),
            backLeftModule.getModulePosition(),
            backRightModule.getModulePosition()
        };
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return constants_Drive.kDriveKinematics.toChassisSpeeds(
            frontLeftModule.getModuleState(),
            frontRightModule.getModuleState(),
            backLeftModule.getModuleState(),
            backRightModule.getModuleState());
    }

    public void setModuleStates(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = constants_Drive.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants_Drive.MAX_SPEED_METERS_PER_SEC);
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    
    //face forward method. Called once the bot is enabled
    public void faceAllFoward() {
        backRightModule.wheelFaceForward();
        frontLeftModule.wheelFaceForward();
        frontRightModule.wheelFaceForward();
        backLeftModule.wheelFaceForward();
        System.out.println("exacuted faceAll");
    }
    
    public Command resetWheels(){
        return runOnce(() -> {
            faceAllFoward();
        });
    }

    //stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
        
    public Command fieldOrientedToggle(){
        return runOnce(() -> {fieldOriented = !fieldOriented;});
    }
    
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());

        //multiple debugging values are listed here. Names are self explanitory
        
        //Odometer and other gyro values
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);
        
        //AE Degrees Reading
        SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getABSPosition());
        SmartDashboard.putNumber("Back Right AE Value", backRightModule.getABSPosition());
        SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getABSPosition());
        SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getABSPosition());

        //RE Degrees Reading
        SmartDashboard.putNumber("Back left RE Value", backLeftModule.getModulePosition().angle.getDegrees());
        SmartDashboard.putNumber("Back Right RE Value", backRightModule.getModulePosition().angle.getDegrees());
        SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getModulePosition().angle.getDegrees());
        SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getModulePosition().angle.getDegrees());
        //RE Distance Reading
        SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
        SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
        SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
        SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());

        
    

    //  flModPos = new SwerveModulePosition(frontLeftModule.getDrivePosition(), frontLeftModule.getSteerState().angle);
    //  frModPos = new SwerveModulePosition(frontRightModule.getDrivePosition(), frontRightModule.getSteerState().angle);
    //  blModPos = new SwerveModulePosition(backLeftModule.getDrivePosition(), backLeftModule.getSteerState().angle);
    //  brModPos = new SwerveModulePosition(backRightModule.getDrivePosition(), backRightModule.getSteerState().angle);
        }
}