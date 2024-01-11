package frc.robot.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleStateU extends SwerveModuleState {

    private static final PIDController utilityController = new PIDController(0.0, 0.0, 0.0);

    public double drivePositionRad;

    public SwerveModuleStateU() { 
        super();
        utilityController.enableContinuousInput(0, 360);
        drivePositionRad = 0;
     }

    public SwerveModuleStateU(double speedMetersPerSecond, Rotation2d angle) {
        super(speedMetersPerSecond, angle);
        this.drivePositionRad = 0;
        utilityController.enableContinuousInput(0, 360);
    }

    public SwerveModuleStateU(double speedMetersPerSecond, Rotation2d angle, double drivePositionRad) {
        super(speedMetersPerSecond, angle);
        this.drivePositionRad = drivePositionRad;
        utilityController.enableContinuousInput(0, 360);
    }

    public SwerveModuleStateU(SwerveModuleState state) {
      super( state.speedMetersPerSecond, state.angle );
      this.drivePositionRad = 0;
      utilityController.enableContinuousInput(0, 360);
    }

    public static SwerveModuleStateU optimize(
      SwerveModuleStateU desiredState, Rotation2d currentAngle) {

        double option1 = desiredState.angle.getDegrees();
        double option2 = desiredState.angle.getDegrees() - 180;

        if(option2 < 0) {
          option2 += 360;
        }

        utilityController.setSetpoint(option1);
        utilityController.calculate(currentAngle.getDegrees());

        double delta1 = utilityController.getPositionError();
        
        utilityController.setSetpoint(option2);
        utilityController.calculate(currentAngle.getDegrees());

        double delta2 = utilityController.getPositionError();        

        return (Math.abs(delta1) < Math.abs(delta2)) ? new SwerveModuleStateU(desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(option1)) : 
                new SwerveModuleStateU(-desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(option2));
        

    // var delta = desiredState.angle.minus(currentAngle);
    // if (Math.abs(delta.getDegrees()) > 90.0) {
    //   return new SwerveModuleStateU(
    //       -desiredState.speedMetersPerSecond,
    //       desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)), desiredState.drivePositionRad);
    // } else {
    //   return new SwerveModuleStateU(desiredState.speedMetersPerSecond, desiredState.angle, desiredState.drivePositionRad);
    // }
  }
    
    public static SwerveModuleStateU[] toModuleStatesU(SwerveModuleState[] swerveModuleStates) {

      SwerveModuleStateU[] uStates = new SwerveModuleStateU[4];

      uStates[0] =  new SwerveModuleStateU(swerveModuleStates[0]);
      uStates[1] = new SwerveModuleStateU(swerveModuleStates[1]);
      uStates[2] =  new SwerveModuleStateU(swerveModuleStates[2]);
      uStates[3] = new SwerveModuleStateU(swerveModuleStates[3]);

      return uStates;
    }
}
