package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.ConstantsF.RobotConstants;


/**
 * Class for Swerve Drive Odometry. Uses Position Delta measurements instead of
 * the WPILib Velocity Method to reduce Drift over time. Odometry (X,Y) = (0,0) is the bottom left corner of the field.
 * Rotations are Counter-Clockwise Positive
 */
public class OdometryU {

    private SwerveModuleStateU[] prevModuleStates = new SwerveModuleStateU[4];
    private Rotation2d prevGyroAngle;

    private Pose2d poseMeters;
    private final SwerveDriveKinematics kinematics = RobotConstants.swerveKinematics;

    private Rotation2d simGyroAngleChange;

    /**
     * Instantiates the Odometry Object with initial position (0,0,0) (X,Y,Theta) on the field.
     * 
     * @param initialModuleStates Initial States of the Swerve Drivetrain's modules
     */
    public OdometryU(SwerveModuleStateU[] initialModuleStates) {
        this(new Vec3d(0,0,0), new Rotation2d(), initialModuleStates);
    }

    /**
     * Instantiates the Odometry Object with designated initial position on the field.
     * @param initialPositionVector Starting Position of the Robot on the Field
     * @param initialGyroAngle Initial Gyro reading
     * @param initialModuleStates Initial States of the Swerve Drivetrain's modules
     */
    public OdometryU(Vec3d initialPositionVector, Rotation2d initialGyroAngle, SwerveModuleStateU[] initialModuleStates) {
        
        poseMeters = new Pose2d(initialPositionVector.X(), initialPositionVector.Y(), initialPositionVector.Theta());
        prevGyroAngle = initialPositionVector.Theta();

        prevModuleStates = initialModuleStates;
    }

    /**
     * Arbitrarily sets the robot's position on the field.
     * 
     * <p> Exact same functionality as the setPosition(Vec3d pose, Rotation2d gyroAngle) function, just with a different
     * name to clarify what the goal is when the function is called
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     * 
     * <p>Do not call this function then reset the gyro afterwards, as the measured angle offset will now be incorrect.
     *
     * @param positionVector The position on the field that your robot is at.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates Module states of the drivetrain
     */
    public void resetPosition(Vec3d positionVector, Rotation2d gyroAngle, SwerveModuleStateU[] moduleStates) {
        prevModuleStates = moduleStates;
        poseMeters = positionVector.asPose2d();
        prevGyroAngle = gyroAngle;
    }

    /**
     * Arbitrarily sets the robot's position on the field.
     * 
     * <p> Exact same functionality as the resetPosition(Vec3d pose, Rotation2d gyroAngle) function, just with a different
     * name to clarify what the goal is when the function is called
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     * 
     * <p>Do not call this function then reset the gyro afterwards, as the measured angle offset will now be incorrect.
     *
     * @param positionVector The position on the field that your robot is at.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates Module states of the drivetrain
     */
    public void setPosition(Vec3d positionVector, Rotation2d gyroAngle, SwerveModuleStateU[] moduleStates) {
        prevModuleStates = moduleStates;
        poseMeters = positionVector.asPose2d();
        prevGyroAngle = gyroAngle;
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
        return poseMeters;
    }
    

    /**
    * Updates the robot's position on the field using forward kinematics and integration of the pose
    * over time. This method automatically calculates the current time to calculate period
    * (difference between two timestamps). This also takes in an angle parameter which is 
    * used instead of the angular rate that is calculated from forward kinematics.
    *
    * @param gyroAngle The angle reported by the gyroscope.
    * @param moduleStates The current state of all swerve modules. Please provide the states in the
    *     same order in which you instantiated your SwerveDriveKinematics.
    */
    public void update(Rotation2d gyroAngle, SwerveModuleStateU... moduleStates) {

        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {

            measuredStatesDiff[i] = new SwerveModuleStateU(
                (moduleStates[i].drivePositionRad - prevModuleStates[i].drivePositionRad)
                    * RobotConstants.swerveWheelRadius_Meters,
                moduleStates[i].angle);
            
        }

        prevModuleStates = moduleStates;

        ChassisSpeeds chassisStateDiff =
            kinematics.toChassisSpeeds(measuredStatesDiff);

        double angleDiff = gyroAngle.getDegrees() - prevGyroAngle.getDegrees();

        if (angleDiff > 180)
        {
            angleDiff -= 360;
        } else if ( angleDiff < -180 ) {
            angleDiff += 360;
        }

        poseMeters = poseMeters.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
                                chassisStateDiff.vyMetersPerSecond,
                                Math.toRadians(angleDiff)));

        prevGyroAngle = gyroAngle;

        simGyroAngleChange = Rotation2d.fromDegrees(Math.toDegrees(kinematics.toChassisSpeeds(moduleStates).omegaRadiansPerSecond) * 0.2) ;

    }

    public Rotation2d getSimulatedGyroAngleChange() {
        return simGyroAngleChange;
    }
    
}
