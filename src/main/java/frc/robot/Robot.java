// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Simulation.BatterySimulator;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Util.SwerveModule;
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

    //Change SIM to DRIVE
    //Also for whatever reason gyro.calibrarte() isnt a thing anymore but its fine for now (hopefully)
    //and the gyro auto-calibrates at the start anyways (source: an insane amount of copium)
    //also worst case scenario just make it move by directly setting the motors in autonomous periodic
    Drivetrain drivetrain = new Drivetrain( Drivetrain.OperatingMode.DRIVE ); //Instantiate the DriveTrain in sim mode
    private Field2d m_field = new Field2d(); //WPI Lib class that handles and manages the simulated field

    XboxController cont = new XboxController(0);

    @Override
    public void robotInit() {
      
      SmartDashboard.putData("Field", m_field); //Upload field to SmartDashboard
      drivetrain.resetHeading();
      drivetrain.resetOdometry( new Vec3d(3.0, 3.0, 0) ); //Resets the Robot's position.
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

      SwerveModuleStateU[] states = drivetrain.getSwerveModuleStates();

      for(int i = 0; i < 4; i++)
      {
        SmartDashboard.putNumber("angle " + (i + 1), states[i].angle.getDegrees());
      }

      SwerveModule[] mods = drivetrain.getModules();

      for(int i = 0; i < 4; i++)
      {
        SmartDashboard.putNumber("desired " + (i + 1), mods[i].desired);
      }

      for(int i = 0; i < 4; i++)
      {
        SmartDashboard.putNumber("speed " + (i + 1), mods[i].turnSpeed);
      }
    }

    @Override
    public void autonomousInit() {
      drivetrain.resetOdometry( new Vec3d(0.0, 3.0, 0) );
    }

    @Override
    public void autonomousPeriodic()
    {
      drivetrain.set(new ChassisSpeeds(0.1, 0, 0), false);
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic()
    {
      drivetrain.set(new ChassisSpeeds(-cont.getRawAxis(1), -cont.getRawAxis(0), cont.getRawAxis(2)), false);
      //drivetrain.set(new ChassisSpeeds(0, 0, 0.3), false);
      //SmartDashboard.putNumber("Heading", drivetrain.getHeading().getDegrees());
      //SmartDashboard
    }

    @Override
    public void disabledInit() {
      drivetrain.set( new ChassisSpeeds(0.0, 0.0, 0.0) , false);
    }

    @Override
    public void disabledPeriodic() {
      drivetrain.set( new ChassisSpeeds(0.0, 0.0, 0.0) , false);
    }

    double testStartTime = 0;

    @Override
    public void testInit() {
      testStartTime = RobotController.getFPGATime();
    }

    @Override
    public void testPeriodic() {
      drivetrain.set( new ChassisSpeeds(0, 0.0, 0.0) , false);
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
