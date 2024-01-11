package frc.robot.Hardware;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class SparkMaxU extends CANSparkMax {

    private final Notifier notifier;
    private final CAN canInterface;
    private final LinearFilter velocityFilter;
    private final int averagingTaps = 5;

    private final int deviceManufacturer = 5; //REV
    private final int deviceType = 2; //Spark Max
    private final int apiId = 98; //Periodic Status 2

    private final PIDController velocityController;

    private boolean firstCycle = true;
    private boolean enabled = false;
    private double ffVolts = 0.0;
    private double timestamp = 0.0;
    private double position = 0.0;
    private double velocity = 0.0;

    public SparkMaxU(int deviceId, MotorType type) {
        super(deviceId, type);

        this.canInterface = new CAN(this.getDeviceId(), this.deviceManufacturer, this.deviceType);
        this.velocityFilter = LinearFilter.movingAverage(this.averagingTaps);

        this.velocityController = new PIDController(0.0, 0.0, 0.0, 0.02);

        this.notifier = new Notifier(this::update);
        this.notifier.startPeriodic(0.02);
    }

    private void update() {
        CANData canData = new CANData();
        boolean isFresh = canInterface.readPacketNew(apiId, canData);
        double newTimestamp = canData.timestamp / 1000.0;
        double newPosition = ByteBuffer.wrap(canData.data)
            .order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);
    
        if (isFresh) {
            synchronized (this) {
                if (!this.firstCycle) {
                velocity = this.velocityFilter.calculate(
                    (newPosition - position) / (newTimestamp - timestamp) * 60);
                }
                firstCycle = false;
                timestamp = newTimestamp;
                position = newPosition;
        
                if (DriverStation.isDisabled()) {
                enabled = false;
                this.stopMotor();
                }
                if (enabled) {
                this.setVoltage(ffVolts + this.velocityController.calculate(velocity));
                }
            }
        }
    }

    public synchronized void setReference(double velocityRpm, double ffVolts) {
        velocityController.setSetpoint(velocityRpm);
        this.ffVolts = ffVolts;
    
        if (!enabled) {
          velocityController.reset();
        }
        enabled = true;
      }
    
      /**
       * Disables the controller. No further commands will be sent to the Spark Max, but the
       * measurements will continue to update.
       */
      public synchronized void disable() {
        if (enabled) {
          this.stopMotor();
        }
        enabled = false;
      }
    
      /** Sets the PID gains. */
      public synchronized void setPID(double kp, double ki, double kd) {
        this.velocityController.setPID(kp, ki, kd);
      }
    
      /**
       * Returns the current position in rotations.
       */
      public synchronized double getPosition() {
        return this.position;
      }
    
      /**
       * Returns the current velocity in rotations/minute.
       */
      public synchronized double getVelocity() {
        return this.velocity;
      }
    
}
