package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Vision;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Drivetrain.OperatingMode;
import frc.robot.Util.OdometryU;
import frc.robot.Util.SwerveModuleStateU;
import frc.robot.Util.Vec3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VSLAM {

    private Vec3d RobotPose;

    private final AHRS gyro;
    private final AnalogGyro simulated_gyro = new AnalogGyro(1);
    private AnalogGyroSim m_gyroSim = new AnalogGyroSim(simulated_gyro);

    private final Drivetrain.OperatingMode dMode;

    private final Vision vision;
    private final OdometryU odometry;

    public VSLAM( Drivetrain.OperatingMode operatingMode, SwerveModuleStateU[] initiaModuleStates ) {

        this.dMode = operatingMode;

        if(operatingMode == OperatingMode.SIM) {
            gyro = null;
            vision = null;

            resetHeading();
        } else {

            gyro = new AHRS(SPI.Port.kMXP);

            gyro.calibrate();
            while(gyro.isCalibrating()) {}

            DriverStation.reportError("Gyro Successfully Callibrated", false);

            gyro.reset();

            vision = new Vision(RobotConstants.cameraInstanceName);
        }

        odometry = new OdometryU(new Vec3d(), getHeading(), initiaModuleStates);
        RobotPose = Vec3d.fromPose2d(odometry.getPoseMeters());

    }

    public void updateOdometry(SwerveModuleStateU[] moduleStates) {

        if(vision != null) {
            vision.updateVision();

            if(!vision.hasTargets()) {
                odometry.update( getHeading(), moduleStates );
    
                RobotPose = Vec3d.fromPose2d(odometry.getPoseMeters()) ;
            } else {
                
            }
        } else {
            odometry.update( getHeading(), moduleStates );
    
            RobotPose = Vec3d.fromPose2d(odometry.getPoseMeters()) ;

            updateSimulatedGyroHeading();
        }

    }

    public void resetOdometry(SwerveModuleStateU[] moduleStates) {
        resetOdometry(new Vec3d(), new Rotation2d(), moduleStates);
    }
   
    public void resetOdometry(Vec3d initalPose, Rotation2d gyroAngle, SwerveModuleStateU[] moduleStates) {
        odometry.resetPosition(initalPose, gyroAngle, moduleStates);
    }

    public void resetHeading() {
        if(this.dMode == OperatingMode.DRIVE) 
        {
            gyro.reset();
        } 
        else if(this.dMode == OperatingMode.SIM) 
        {
            m_gyroSim.resetData();
        }
    }


    public Vec3d getRobotPoseMeters() {
        return RobotPose;
    }

    public Rotation2d getHeading() {

        if(this.dMode == OperatingMode.DRIVE) 
        {
            return gyro.getRotation2d();
        } 
        else if(this.dMode == OperatingMode.SIM) 
        {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        }
        else 
        {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        }

    }

    public void updateSimulatedGyroHeading() {
        double angle = m_gyroSim.getAngle();
        
        angle -= odometry.getSimulatedGyroAngleChange().getDegrees();

        while (angle < 0) {
            angle += 360;
        }
        while (angle > 360) {
            angle -= 360;
        }

        m_gyroSim.setAngle(angle);

        SmartDashboard.putNumber("angle", m_gyroSim.getAngle());
    }
    
}
