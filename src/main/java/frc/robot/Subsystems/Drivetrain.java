package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.ConstantsF.PortConstants;
import frc.robot.ConstantsF.RobotConstants;
import frc.robot.Simulation.SimSwerveModule;
import frc.robot.Util.SwerveModule;
import frc.robot.Util.SwerveModuleStateU;
import frc.robot.Util.Vec3d;

public class Drivetrain {
    
    private final SwerveModule[] swerveModules;

    private SwerveModuleStateU[] moduleStates = new SwerveModuleStateU[4];

    private final OperatingMode dMode;

    private final VSLAM vslam;
    private final SwerveDriveKinematics kinematics = RobotConstants.swerveKinematics;

    /**
     * Instantiates a DriveTrain Object. Can be in SIM mode or DRIVE mode.
     * @param DriveTrainMode Directs what mode the drivetrain is instantiated and operated in
     */
    public Drivetrain(OperatingMode DriveTrainMode)
    {

        this.dMode = DriveTrainMode;

        switch( dMode ) {
            case DRIVE:
                swerveModules = new SwerveModule[]{ new SwerveModule(PortConstants.FrontLeftThrottlePort,
                                                                    PortConstants.FrontLeftAnglePort,
                                                                    PortConstants.FrontLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetFL),
                                                    new SwerveModule(PortConstants.FrontRightThrottlePort,
                                                                    PortConstants.FrontRightAnglePort,
                                                                    PortConstants.FrontRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindLeftThrottlePort,
                                                                    PortConstants.HindLeftAnglePort,
                                                                    PortConstants.HindLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindRightThrottlePort,
                                                                    PortConstants.HindRightAnglePort,
                                                                    PortConstants.HindRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHR),
                                                };
                break;
            case SIM:
                swerveModules = new SwerveModule[]{ new SimSwerveModule(),
                                                    new SimSwerveModule(),
                                                    new SimSwerveModule(),
                                                    new SimSwerveModule()
                                                };
                break;
            default:
                swerveModules = new SwerveModule[]{ new SwerveModule(PortConstants.FrontLeftThrottlePort,
                                                                    PortConstants.FrontLeftAnglePort,
                                                                    PortConstants.FrontLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetFL),
                                                    new SwerveModule(PortConstants.FrontRightThrottlePort,
                                                                    PortConstants.FrontRightAnglePort,
                                                                    PortConstants.FrontRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindLeftThrottlePort,
                                                                    PortConstants.HindLeftAnglePort,
                                                                    PortConstants.HindLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindRightThrottlePort,
                                                                    PortConstants.HindRightAnglePort,
                                                                    PortConstants.HindRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHR),
                                                };
                break;
        }

        this.vslam = new VSLAM( this.dMode, getSwerveModuleStates() );

        //AutoBuilder.configureHolonomic(null, null, null, null, null, null, null);
    }

    /**
     * Sets the desired module states
     * @param chassisSpeeds
     * @param voltageControl
     */
    public void set(ChassisSpeeds chassisSpeeds, boolean velocityControl) {

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
                                                            chassisSpeeds.vyMetersPerSecond,
                                                            chassisSpeeds.omegaRadiansPerSecond, 
                                                            vslam.getHeading());

        moduleStates = SwerveModuleStateU.toModuleStatesU(kinematics.toSwerveModuleStates(chassisSpeeds));

        setSwerveModuleStates(moduleStates, velocityControl);
        
    }

    public void lol(double speed)
    {
        swerveModules[0].getThrottle().set(speed);
        swerveModules[1].getThrottle().set(speed);
        swerveModules[2].getThrottle().set(speed);
        swerveModules[3].getThrottle().set(speed);
    }

    /**
     * Call to update the VSLAM odometry reading
     */
    public void updateOdometry() {
        vslam.updateOdometry( getSwerveModuleStates() );
    }

    /**
     * Resets robot's position on the field to (0,0,0), (X,Y,Theta)
     */
    public void resetOdometry() {
        resetOdometry(new Vec3d());
    }

    /**
     * Resets robot's position on the field to some initial position vector
     * @param initalPositionVector Robot's starting position on the field
     */
    public void resetOdometry(Vec3d initalPositionVector) {
        vslam.resetOdometry(initalPositionVector, vslam.getHeading(), getSwerveModuleStates());
    }

    public Rotation2d getHeading()
    {
        return vslam.getHeading();
    }

    public void resetHeading()
    {
        vslam.resetHeading();
    }

    /**
     * 
     * @return The robot's position on the field in meters and degrees
     */
    public Vec3d getRobotPosition() {
        return vslam.getRobotPoseMeters();
    }


    /**
     * 
     * @return Array of the Drivetrain's Swerve Module's states according to sensor readings
     */
    public SwerveModuleStateU[] getSwerveModuleStates() {
        return new SwerveModuleStateU[]{
            swerveModules[0].getState(),
            swerveModules[1].getState(),
            swerveModules[2].getState(),
            swerveModules[3].getState()
        };
    }

    public void setSwerveModuleStates(SwerveModuleStateU[] mSwerveModuleStates, boolean velocityControl) {

        if(velocityControl) { SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates,
                                                                        RobotConstants.maxAttainableSpeedMetersPerSecond); }
        else { SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, 1);}

        this.moduleStates = mSwerveModuleStates;
        for(int i = 0; i < 4; i++)
        {
            swerveModules[i].setState(this.moduleStates[i], velocityControl);
        }
    }

    public SwerveModule[] getModules()
    {
        return swerveModules;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void updateSimModules() {
        swerveModules[0].update();
        swerveModules[1].update();
        swerveModules[2].update();
        swerveModules[3].update();
    }


    public OperatingMode getDriveTrainOperatingMode() {
        return this.dMode;
    }


    public static enum OperatingMode {
        SIM, DRIVE;
    }
}
