package frc.robot.Util;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConstantsF.RobotConstants;
import frc.robot.Hardware.SparkMaxU;

/**
 * Class to manage an individual Swerve Module
 */
public class SwerveModule {

    private final SparkMaxU ThrottleMotor;
    private final SparkMaxU AngleMotor;
    private final CANcoder angleEncoder;

    private final PIDController angleController;

    private final double angleOffset;

    // Volts per (radian per second)
    private static final double kDriveKv = RobotConstants.swerveThrottlekV;

    // Volts per (radian per second squared)
    private static final double kDriveKa = RobotConstants.swerveThrottlekA;

    protected final LinearSystem<N1, N1, N1> m_DrivePlant;

    private final KalmanFilter<N1, N1, N1> m_observer;

    private final LinearQuadraticRegulator<N1, N1, N1> m_controller;

    private final LinearSystemLoop<N1, N1, N1> m_loop;

    public double desired;
    public double turnSpeed;

    /**
     * Creates a Swerve Module Object
     * @param ThrottleMotor SparkMaxU Throttle Motor
     * @param AngleMotor SparkMaxU Angle Motor
     */
    public SwerveModule(int ThrottleMotorPort, int AngleMotorPort, int angleEncoderPort, double angleOffset) {
        
        this.ThrottleMotor = new SparkMaxU(ThrottleMotorPort, MotorType.kBrushless);
        this.AngleMotor = new SparkMaxU(AngleMotorPort, MotorType.kBrushless);
        this.angleEncoder = new CANcoder(angleEncoderPort);        

        this.angleController = new PIDController(RobotConstants.swerveAnglePID.P(),
                                                RobotConstants.swerveAnglePID.I(),
                                                RobotConstants.swerveAnglePID.D());

        angleController.enableContinuousInput(-180, 180);
        //angleController.setTolerance(0.5);

        this.angleOffset = angleOffset;

        m_DrivePlant = LinearSystemId.identifyVelocitySystem(kDriveKv, kDriveKa);

        m_observer = new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_DrivePlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);

        m_controller = new LinearQuadraticRegulator<>(
            m_DrivePlant,
            VecBuilder.fill(0.01), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(20.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        m_loop = new LinearSystemLoop<>(m_DrivePlant, m_controller, m_observer, 24.0, 0.020);
    }

    /**
     * Creates a Swerve Module Object (Intended for Simulation Use)
     * @param ThrottleMotor SparkMaxU Throttle Motor
     * @param AngleMotor SparkMaxU Angle Motor
     */
    public SwerveModule() {
        this.ThrottleMotor = null;
        this.AngleMotor = null;

        this.angleEncoder = null;

        this.angleController = new PIDController(RobotConstants.swerveAnglePID.P(),
                                                RobotConstants.swerveAnglePID.I(),
                                                RobotConstants.swerveAnglePID.D());

        this.angleOffset = 0;
        //angleController.enableContinuousInput(-180, 180);
        m_DrivePlant = LinearSystemId.identifyVelocitySystem(kDriveKv, kDriveKa);

        m_observer = new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_DrivePlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);

        m_controller = new LinearQuadraticRegulator<>(
            m_DrivePlant,
            VecBuilder.fill(0.01), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(25.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        m_loop = new LinearSystemLoop<>(m_DrivePlant, m_controller, m_observer, 12.0, 0.020);
    }

    public void update() {}

    /**
    * Sets the swerve module motors' speed and angle. This function will also optimize the angle
    * @param state Desired State as SwerveModuleState Object
    * @param velocityControl set to True if velocity control is desired on the spark max
    */
    public void setState(SwerveModuleStateU state, boolean velocityControl) {

        double currentAngle = this.angleEncoder.getAbsolutePosition().getValueAsDouble() * 360 - angleOffset;

        if(currentAngle > 180)
        {
            currentAngle -= 360;
        }

        if(currentAngle < -180)
        {
            currentAngle += 360;
        }

        SwerveModuleStateU correctedState = SwerveModuleStateU.optimize(state, Rotation2d.fromDegrees(currentAngle));

        if (velocityControl)
        {
            this.ThrottleMotor.setVoltage(calculateVelocityControlVoltage(correctedState.speedMetersPerSecond, ThrottleMotor.getVelocity()*RobotConstants.moduleRPM_to_MetersPerSecond));
        } 
        else
        {
            this.ThrottleMotor.set(correctedState.speedMetersPerSecond);
        }

        this.angleController.setSetpoint(correctedState.angle.getDegrees());
        double temp = this.angleController.calculate(currentAngle);
        this.AngleMotor.set(temp);
        desired = angleController.getSetpoint();
        turnSpeed = AngleMotor.get();
    }

    public double calculateVelocityControlVoltage(double setpoint, double velocity) {
        m_loop.setNextR(VecBuilder.fill(setpoint));
        m_loop.correct(VecBuilder.fill(velocity));
        m_loop.predict(0.020);

        return m_loop.getU(0);
    }

    /**
    * 
    * @return The Module's current state according to sensor readings
    */
    public SwerveModuleStateU getState() {
        double currentAngle = this.angleEncoder.getAbsolutePosition().getValueAsDouble() * 360 - angleOffset;

        if(currentAngle > 180)
        {
            currentAngle -= 360;
        }

        if(currentAngle < -180)
        {
            currentAngle += 360;
        }

        return new SwerveModuleStateU(this.ThrottleMotor.getEncoder().getVelocity() * RobotConstants.moduleRPM_to_MetersPerSecond,
                                    Rotation2d.fromDegrees(currentAngle),
                                    this.ThrottleMotor.getPosition() * 2 * Math.PI );
    }

    /**
    * Gets the Current Swerve Module State, however the velocity and angle can be converted to any convenient unit
    * @param throttleVelocityUnits Desired Output units for the module throttle motor velocity
    * @param angleUnits Desired Output units for the module angle motor position
    * @return SwerveModuleState Object in desired Units
    */
    public SwerveModuleStateU getState(ModuleUnits throttleVelocityUnits, ModuleUnits angleUnits) {

        SwerveModuleStateU currentState = getState();

        double velocity;
        double angle;

        switch(throttleVelocityUnits) {
            case EncoderCounts:
                velocity = currentState.speedMetersPerSecond;
                break;
            case FeetPerSecond:
                velocity = currentState.speedMetersPerSecond * RobotConstants.moduleRPM_to_FeetPerSecond;
                break;
            case MetersPerSecond:
                velocity = currentState.speedMetersPerSecond * RobotConstants.moduleRPM_to_MetersPerSecond;
                break;
            default:
                velocity = currentState.speedMetersPerSecond;
                break;
        }

        switch (angleUnits) {
            case Degrees:
                angle = currentState.angle.getDegrees();
                break;
            case Radians:
                angle = currentState.angle.getRadians();
                break;
            default:
                angle = currentState.angle.getDegrees();
                break;
            
        }

        return new SwerveModuleStateU(velocity, Rotation2d.fromDegrees(angle), this.ThrottleMotor.getPosition() * 2 * Math.PI);
    }
    
    
    /**
     * Unit conversion enum. EncoderCounts, MetersPerSecond, and FeetPerSecond are for velocity. Radians, and Degrees are for angles
     */
    public static enum ModuleUnits {

        MetersPerSecond,
        FeetPerSecond,
        Radians,
        Degrees,
        EncoderCounts
        

    }
    

    public SparkMaxU getThrottle()
    {
        return ThrottleMotor;
    }
}
