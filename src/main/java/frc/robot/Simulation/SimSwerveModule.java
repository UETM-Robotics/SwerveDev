// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConstantsF.RobotConstants;
import frc.robot.Util.SwerveModule;
import frc.robot.Util.SwerveModuleStateU;
import edu.wpi.first.math.MathUtil;

public class SimSwerveModule extends SwerveModule{

    private final double loopPeriodSecs = 0.02;

    private final FlywheelSim driveSim;
    //   new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
        
    private FlywheelSim turnSim =
        new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

    private double drivePositionRad = 0;

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private final PIDController angleController;

    public SimSwerveModule() {
        super();

        driveSim = new FlywheelSim(m_DrivePlant, DCMotor.getNEO(1), 6.75);

        angleController = new PIDController(3.0, 0.0, 0.0);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        BatterySimulator.getInstance().addCurrentDrawSource( () -> this.driveSim.getCurrentDrawAmps() );
        BatterySimulator.getInstance().addCurrentDrawSource( () -> this.turnSim.getCurrentDrawAmps() );
    }

    @Override
    public void update() {

        this.driveSim.update(loopPeriodSecs);
        this.turnSim.update(loopPeriodSecs);

        double angleDiffRad = this.turnSim.getAngularVelocityRadPerSec() * loopPeriodSecs;
        this.turnRelativePositionRad += angleDiffRad;
        this.turnAbsolutePositionRad += angleDiffRad;

        while (this.turnAbsolutePositionRad < -Math.PI) {
            this.turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (this.turnAbsolutePositionRad > Math.PI) {
            this.turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        this.drivePositionRad += this.driveSim.getAngularVelocityRadPerSec() * loopPeriodSecs;

    }

    @Override
    public void setState(SwerveModuleStateU state, boolean velocityControl)
    {

        SwerveModuleStateU correctedState = SwerveModuleStateU.optimize(state, 
                                                            Rotation2d.fromDegrees( Math.toDegrees(this.turnAbsolutePositionRad) ));

        if (velocityControl)
        {
            SmartDashboard.putNumber("reference", correctedState.speedMetersPerSecond);
            SmartDashboard.putNumber("state", this.driveSim.getAngularVelocityRPM()*RobotConstants.moduleRPM_to_MetersPerSecond);
            setDriveVoltage(calculateVelocityControlVoltage(correctedState.speedMetersPerSecond*1.379995584014131, this.driveSim.getAngularVelocityRPM()*RobotConstants.moduleRPM_to_MetersPerSecond ));
        } 
        else
        {
            setDriveVoltage(correctedState.speedMetersPerSecond * RobotController.getBatteryVoltage());
        }

        this.angleController.setSetpoint(correctedState.angle.getRadians());
        setTurnVoltage(this.angleController.calculate(turnAbsolutePositionRad));

        // if(correctedState.speedMetersPerSecond != 0) {
        //     this.angleController.setSetpoint(correctedState.angle.getRadians());
        //     setTurnVoltage(this.angleController.calculate(turnAbsolutePositionRad));
        // } else {
        //     setTurnVoltage(0);
        // }

    }

    /**
     * Returns Simulated Measured Module State. Velocity in Meters Per Second, and Angle in Radians.
     */
    public SwerveModuleStateU getState()
    {
        return new SwerveModuleStateU( driveSim.getAngularVelocityRPM() * RobotConstants.moduleRPM_to_MetersPerSecond,
                                    Rotation2d.fromDegrees(Math.toDegrees(this.turnAbsolutePositionRad)),
                                    this.drivePositionRad );
    }

    public void setDriveVoltage(double volts) 
    {
        this.driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        this.driveSim.setInputVoltage(this.driveAppliedVolts);
    }

    public void setTurnVoltage(double volts) 
    {
        this.turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        this.turnSim.setInputVoltage(this.turnAppliedVolts);
    }
  
}