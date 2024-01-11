// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Simulation.BatterySimulator;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Util.SwerveModuleStateU;
import frc.robot.Util.Vec2d;
import frc.robot.Util.Vec3d;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


    Drivetrain drivetrain = new Drivetrain( Drivetrain.OperatingMode.SIM ); //Instantiate the DriveTrain in sim mode
    private Field2d m_field = new Field2d(); //WPI Lib class that handles and manages the simulated field

    @Override
    public void robotInit() {
      SmartDashboard.putData("Field", m_field); //Upload field to SmartDashboard
      drivetrain.resetOdometry( new Vec3d(3.0, 3.0, -90) ); //Resets the Robot's position.
                                                            //Odometry (X,Y) = (0,0) is the bottom left corner of the field
                                                            //Rotations are Counter-Clockwise Positive
                                                            //This initial position describes a robot 3 meters from the bottom edge,
                                                            //                                        3 meters from the driver station,
                                                            //                                        Rotated 90 degrees Clockwise
    }

    @Override
    public void robotPeriodic() {
      drivetrain.updateOdometry();
      m_field.setRobotPose(drivetrain.getRobotPosition().asPose2d());
    }

    @Override
    public void autonomousInit() {
      drivetrain.resetOdometry( new Vec3d(0.0, 3.0, 0) );
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
      drivetrain.set( new ChassisSpeeds(0.0, 0.0, 0.0) , false);
    }

    @Override
    public void disabledPeriodic() {
      drivetrain.set( new ChassisSpeeds(0.0, 0.0, 0.0) , true);
    }

    double testStartTime = 0;

    @Override
    public void testInit() {
      testStartTime = RobotController.getFPGATime();
    }

    @Override
    public void testPeriodic() {
      drivetrain.set( new ChassisSpeeds(1, 0.0, 0.0) , true);
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {

      BatterySimulator.getInstance().updateSimulatedBatteryVoltage();

      drivetrain.updateSimModules();

      ChassisSpeeds speed = drivetrain.getChassisSpeeds();

      Vec2d xy_velocity = new Vec2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

      SmartDashboard.putNumber("speed", xy_velocity.len());
      SmartDashboard.putNumber("turn speed", speed.omegaRadiansPerSecond);
      
    }
}
