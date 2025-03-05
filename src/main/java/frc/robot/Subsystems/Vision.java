package frc.robot.Subsystems;

import java.util.Optional;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LimelightHelpers;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Util.Constants.constants_Auto;
import frc.robot.Util.Constants.constants_Limelight;
import frc.robot.Util.LimelightHelpers.LimelightResults;

import java.util.function.*;

public class Vision extends SubsystemBase{
    
public Swerve s_swerve;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double ySpeed, xSpeed, turningSpeed, correctionX, correctionZ, correctionT, distanceX, distanceY;
public double[] localizedPose;
public double[] botPose_targetSpace, targetPose_robotSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController xPIDController, yPIDController;
public BooleanSupplier interrupted;
public String limelight_Coral = "limelight-coral";
public String limelight_Algae = "liemlight-algae";

public LimelightResults r_limelight;

public LimelightHelpers.PoseEstimate mt2;


// public Apri fieldLayout;

// public SlewRateLimiter tLimiter, xLimiter, zLimiter;

    public Vision(Swerve s_swerve){
        this.s_swerve = s_swerve;
        networkTables = NetworkTableInstance.getDefault().getTable("limelight");
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        
        
        thetaPIDController = new ProfiledPIDController(constants_Limelight.THETA_P, constants_Limelight.THETA_I, constants_Limelight.THETA_D, constants_Auto.THETA_CONTROLLER_CONSTRAINTS);
        xPIDController = new ProfiledPIDController(constants_Limelight.LINEAR_P, constants_Limelight.LINEAR_I, constants_Limelight.LINEAR_D, constants_Auto.LINEAR_CONSTRAINTS);
        yPIDController = new ProfiledPIDController(constants_Limelight.LINEAR_P, constants_Limelight.LINEAR_I, constants_Limelight.LINEAR_D, constants_Auto.LINEAR_CONSTRAINTS);
        setPIDControllers();
    }

    // public Optional<Pose2d> getPoseFromAprilTags() {
    //     double[] botpose = localizedPose;
    //     if(botpose.length < 7 || targetID_Coral == -1) return Optional.empty();
    //     return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    // }
  

    public void setPIDControllers()
    {
        thetaPIDController.setGoal(0);//radians
        thetaPIDController.setTolerance(Math.toRadians(1)); //radians

        xPIDController.setGoal(0);//meters
        xPIDController.setTolerance(0.1);//meters

        yPIDController.setGoal(1);//meters
        yPIDController.setTolerance(0.05);//meters
    }

    
    public void resetDriveValues()
    {
        turningSpeed = 0;
        ySpeed = 0;
    }

    public boolean atGoal()
    {
        return yPIDController.atGoal() && thetaPIDController.atGoal();
    }


    public Command autoAlgae = new Command()
    {
        
        @Override
        public void initialize()
        {
            s_swerve.faceAllFoward();
        }
        
        @Override
        public void execute()
        {
            turningSpeed = thetaPIDController.calculate(getXAng_Rad(limelight_Algae)); /*Rads / sec */
            ySpeed = -1 * yPIDController.calculate(distanceY); /*m/sec */
            s_swerve.setModuleStates(new ChassisSpeeds(-ySpeed, 0, turningSpeed));
        }

        @Override
        public boolean isFinished()
        {
            return atGoal() || !hasTarget(limelight_Algae);
        }

        @Override
        public void end(boolean interrupted)
        {
            resetDriveValues();
        }
    };

    public Command autoProcessor = new Command()
    {
        
        @Override
        public void initialize()
        {
            s_swerve.faceAllFoward();
        }
        
        @Override
        public void execute()
        {
            turningSpeed = thetaPIDController.calculate(getXAng_Rad(limelight_Algae)); /*Rads / sec */
            ySpeed = -1 * yPIDController.calculate(distanceY); /*m/sec */
            s_swerve.setModuleStates(new ChassisSpeeds(-ySpeed, 0, turningSpeed));
        }

        @Override
        public boolean isFinished()
        {
            return atGoal() || !hasTarget(limelight_Algae);
        }

        @Override
        public void end(boolean interrupted)
        {
            resetDriveValues();
        }
    };

    public Command autoReef = new Command()
    {
        
        @Override
        public void initialize()
        {
            s_swerve.faceAllFoward();
        }
        
        @Override
        public void execute()
        {
            turningSpeed = thetaPIDController.calculate(getXAng_Rad(limelight_Algae)); /*Rads / sec */
            // ySpeed = -1 * yPIDController.calculate(distanceY); /*m/sec */
            s_swerve.setModuleStates(new ChassisSpeeds(0, 0, turningSpeed));
        }

        @Override
        public boolean isFinished()
        {
            return atGoal() || !hasTarget(limelight_Algae);
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

    //         // ySpeed = -1 * yPIDController.calculate(distanceY); /*m/sec */
    //         // s_swerve.setModuleStates(new ChassisSpeeds(ySpeed, 0, turningSpeed));
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
        return Math.toRadians(LimelightHelpers.getTX(camera));
    }

    public double getYAng_Rad(String camera)
    {
        return Math.toRadians(LimelightHelpers.getTY(camera));
    }

    public boolean hasTarget(String camera)
    {
        return LimelightHelpers.getTV(camera);
    }

    public double distanceY(String camera)
    {
        return (((13 - constants_Limelight.getCameraList(camera).get(3)) / (Math.tan(Math.toRadians(getYAng_Rad(camera)+constants_Limelight.getCameraList(camera).get(0))))) + constants_Limelight.getCameraList(camera).get(1)) * 0.0254; //meters from target to center of robot
    }

    // public double distanceX(String camera)
    // {
    //     return (Math.cos(Math.toRadians(distanceY/xAng_Coral))) * 0.0254;//meters to center of robot
    // }

    @Override
    public void periodic()
    {
        
        // botPose_targetSpace = LimelightHelpers.getBotPose_TargetSpace(limelight_Coral);
        // targetPose_robotSpace = LimelightHelpers.getTargetPose_RobotSpace(limelight_Coral);
        // localizedPose = LimelightHelpers.getBotPose_wpiBlue(limelight_Coral);
        LimelightHelpers.setPipelineIndex(limelight_Coral, 0);
        LimelightHelpers.setPipelineIndex(limelight_Algae, 0);
        
        // distanceY = (((13 - constants_Limelight.HEIGHT_CORAL) / (Math.tan(Math.toRadians(yAng_Coral+constants_Limelight.ANGLE_CORAL)))) + constants_Limelight.DISTANCE_FORWARD_CORAL) * 0.0254; //meters from target to center of robot
        // distanceX = (Math.cos(Math.toRadians(distanceY/xAng_Coral))) * 0.0254;//meters to center of robot
        


        // s_swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // s_swerve.m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);

        // SmartDashboard.putNumber("Distance Horizontal", distanceX);
        // SmartDashboard.putNumber("Distance Forward", distanceY);
        // SmartDashboard.putNumber("Turning Speed", turningSpeed);
        // // SmartDashboard.putNumber("TA Value", targetArea);
        // // SmartDashboard.putNumber("X Speed", xSpeed);
        // SmartDashboard.putBoolean("Has Targets", hasTargets_Coral);
        // SmartDashboard.putNumber("Y Speed", ySpeed);
        // SmartDashboard.putNumber("TX Value", xAng_Coral);
        // SmartDashboard.putNumber("TY Value", yAng_Coral);

        // SmartDashboard.putBoolean("autoDrive", autoDriveToggle);

        // SmartDashboard.putBoolean("Thetha goal", thetaPIDController.atSetpoint());
        // SmartDashboard.putBoolean("Linear goal", yPIDController.atSetpoint());

        // SmartDashboard.putString("Pose Estimate", mt2.pose.toString());


        // SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
        // SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
        // SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
