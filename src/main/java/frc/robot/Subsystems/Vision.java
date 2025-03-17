package frc.robot.Subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LimelightHelpers;
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Util.Constants.constants_Auto;
import frc.robot.Util.Constants.constants_Limelight;
import frc.robot.Util.Controllers;
import frc.robot.Util.LimelightHelpers.LimelightResults;

import java.util.function.*;

public class Vision extends SubsystemBase{
    
public DriveCommand c_Drive;
public Drive s_Drive;
public Controllers u_Controllers;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double xSpeed, turningSpeed;
public double[] localizedPose;
public double[] botPose_targetSpace, targetPose_robotSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController xPIDController;
public BooleanSupplier interrupted;
public String limelight_Coral = "limelight-coral";
public String limelight_Algae = "limelight-algae";

public LimelightResults r_limelight;

public LimelightHelpers.PoseEstimate mt2;


// public Apri fieldLayout;

// public SlewRateLimiter tLimiter, xLimiter, zLimiter;

    public Vision(DriveCommand c_Drive, Drive s_Drive, Controllers u_Controllers){
        this.c_Drive = c_Drive;
        this.s_Drive = s_Drive;
        this.u_Controllers = u_Controllers;
        networkTables = NetworkTableInstance.getDefault().getTable("limelight");
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        
        
        thetaPIDController = new ProfiledPIDController(constants_Limelight.THETA_P, constants_Limelight.THETA_I, constants_Limelight.THETA_D, constants_Auto.THETA_CONTROLLER_CONSTRAINTS);
        xPIDController = new ProfiledPIDController(constants_Limelight.LINEAR_P, constants_Limelight.LINEAR_I, constants_Limelight.LINEAR_D, constants_Auto.LINEAR_CONSTRAINTS);
        setPIDControllers();
        LimelightHelpers.setCameraPose_RobotSpace(limelight_Algae, constants_Limelight.getCameraList(limelight_Algae).get(1), constants_Limelight.getCameraList(limelight_Algae).get(2), constants_Limelight.getCameraList(limelight_Algae).get(3), 180, constants_Limelight.getCameraList(limelight_Algae).get(0), 0);
        LimelightHelpers.setCameraPose_RobotSpace(limelight_Coral, constants_Limelight.getCameraList(limelight_Coral).get(1), constants_Limelight.getCameraList(limelight_Coral).get(2), constants_Limelight.getCameraList(limelight_Coral).get(3), 90, constants_Limelight.getCameraList(limelight_Coral).get(0), 0);

    }

    // public Optional<Pose2d> getPoseFromAprilTags() {
    //     double[] botpose = localizedPose;
    //     if(botpose.length < 7 || targetID_Coral == -1) return Optional.empty();
    //     return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    // }
  

    public void setPIDControllers()
    {
        thetaPIDController.setGoal(0);//radians
        thetaPIDController.setTolerance(Math.toRadians(0.5)); //radians


        xPIDController.setGoal(0);//meters
        xPIDController.setTolerance(0.023);//meters
    }

    
    public void resetDriveValues()
    {
        turningSpeed = 0;
        xSpeed = 0;
    }

    public boolean atGoal()
    {
        return xPIDController.atGoal() && thetaPIDController.atGoal();
    }


    public Command autoAlgae = new Command()
    {
        
        @Override
        public void initialize()
        {
            // s_swerve.faceAllFoward();
        }
        
        @Override
        public void execute()
        {
            turningSpeed = thetaPIDController.calculate(getXAng_Rad(limelight_Algae)); /*Rads / sec */
            xSpeed = xPIDController.calculate(distanceX(limelight_Algae, "Algae")); /*m/sec */
            // c_Drive.driveAlgae(xSpeed, turningSpeed);
        }

        @Override
        public boolean isFinished()
        {
            return !u_Controllers.leftStick.button(4).getAsBoolean() || !hasTarget(limelight_Algae);
        }

        @Override
        public void end(boolean interrupted)
        {
            resetDriveValues();
        }
    };

    // public Command autoProcessor = new Command()
    // {
        
    //     @Override
    //     public void initialize()
    //     {
    //         s_swerve.faceAllFoward();
    //     }
        
    //     @Override
    //     public void execute()
    //     {
        //     turningSpeed = thetaPIDController.calculate(getXAng_Rad(limelight_Algae)); /*Rads / sec */
        //     xSpeed = xPIDController.calculate(distanceX(limelight_Algae, "Processor")); /*m/sec */
        //     c_Drive.driveAlgae(xSpeed, turningSpeed);
        // }

    //     @Override
    //     public boolean isFinished()
    //     {
    //         return atGoal() || !hasTarget(limelight_Algae);
    //     }

    //     @Override
    //     public void end(boolean interrupted)
    //     {
    //         resetDriveValues();
    //     }
    // };

    public Command autoReef = new Command()
    {
        
        @Override
        public void initialize()
        {
            // s_swerve.faceAllFoward();
        }
        
        @Override
        public void execute()
        {
            turningSpeed = thetaPIDController.calculate(getYAng_Rad(limelight_Coral)); /*Rads / sec */
            // c_Drive.driveReef(turningSpeed);
        }

        @Override
        public boolean isFinished()
        {
            return !u_Controllers.leftStick.button(4).getAsBoolean() || !hasTarget(limelight_Coral);
        }

        @Override
        public void end(boolean interrupted)
        {
            resetDriveValues();
        }
    };

    // public Command autoSource = new Command()
    // {
        
    //     @Override
    //     public void initialize()
    //     {
    //         s_swerve.faceAllFoward();
    //     }
        
    //     @Override
    //     public void execute()
    //     {

    //         // ySpeed = -1 * xPIDController.calculate(distanceY); /*m/sec */
    //     }

    //     @Override
    //     public boolean isFinished()
    //     {
    //         return atGoal() || !hasTarget(limelight_Algae);
    //     }

    //     @Override
    //     public void end(boolean interrupted)
    //     {
    //         resetDriveValues();
    //     }
    // };

    



    public double getXAng_Rad(String camera)
    {
        return -1 * Math.toRadians(LimelightHelpers.getTX(camera));
    }

    public double getYAng_Rad(String camera)
    {
        return -1* Math.toRadians(LimelightHelpers.getTY(camera));
    }

    public boolean hasTarget(String camera)
    {
        return LimelightHelpers.getTV(camera);
    }

    public double distanceX(String camera, String target)
    {
        return ((constants_Limelight.getTargetHeight(target) - constants_Limelight.getCameraList(camera).get(3)) / (Math.tan((getYAng_Rad(camera)+constants_Limelight.getCameraList(camera).get(0)))) + constants_Limelight.getCameraList(camera).get(1)); //meters from target to center of robot
    }

    // public double distanceX(String camera)
    // {
    //     return (Math.cos(Math.toRadians(distanceY/xAng_Coral))) * 0.0254;//meters to center of robot
    // }

    @Override
    public void periodic()
    {
        
        LimelightHelpers.setPipelineIndex(limelight_Coral, 0);
        LimelightHelpers.setPipelineIndex(limelight_Algae, 0);
        
        // SmartDashboard.putNumberArray(limelight_Algae + " Debugging values", new Double[]{getXAng_Rad(limelight_Algae), getYAng_Rad(limelight_Algae),turningSpeed});
        // SmartDashboard.putNumberArray(limelight_Coral + " Debugging values", new Double[]{getXAng_Rad(limelight_Coral), getYAng_Rad(limelight_Coral),turningSpeed, ySpeed});
        
        SmartDashboard.putNumber("Turning Speed", turningSpeed);
        SmartDashboard.putNumber("XSpeed", xSpeed);

        SmartDashboard.putNumber("X distance" , distanceX(limelight_Algae, "Algae"));

        // distanceY = (((13 - constants_Limelight.HEIGHT_CORAL) / (Math.tan(Math.toRadians(yAng_Coral+constants_Limelight.ANGLE_CORAL)))) + constants_Limelight.DISTANCE_FORWARD_CORAL) * 0.0254; //meters from target to center of robot
        // distanceX = (Math.cos(Math.toRadians(distanceY/xAng_Coral))) * 0.0254;//meters to center of robot
 
        // s_swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // s_swerve.m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        // SmartDashboard.putString("Pose Estimate", mt2.pose.toString());
        // SmartDashboard.putBoolean("Has Targets", hasTargets_Coral);
        // botPose_targetSpace = LimelightHelpers.getBotPose_TargetSpace(limelight_Coral);
        // targetPose_robotSpace = LimelightHelpers.getTargetPose_RobotSpace(limelight_Coral);
        // localizedPose = LimelightHelpers.getBotPose_wpiBlue(limelight_Coral);
        // SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
        // SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
        // SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
