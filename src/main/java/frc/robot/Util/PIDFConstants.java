package frc.robot.Util;

/**
 * Container Class to manage PIDF Constants
 */
public class PIDFConstants {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    /**
     * Creates PIDF Constants Object with kP, kI, kD, and kF all = 0
     */
    public PIDFConstants() {
        kP = 0;
        kI = 0;
        kD = 0;
        kF = 0;
    }

    /**
     * Creates PIDF Constants Object using only PID. kF is left at 0
     * @param kP Constant for the P term
     * @param kI Constant for the I term
     * @param kD Constant for the D term
     */
    public PIDFConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
    }

    /**
     * Creates PIDF Constants Object
     * @param kP Constant for the P term
     * @param kI Constant for the I term
     * @param kD Constant for the D term
     * @param kF Constant for the F term
     */
    public PIDFConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * 
     * @return kP
     */
    public double P() {
        return kP;
    }

    /**
     * 
     * @return kI
     */
    public double I() {
        return kI;
    }

    /**
     * 
     * @return kD
     */
    public double D() {
        return kD;
    }

    /**
     * 
     * @return kF
     */
    public double F() {
        return kF;
    }
    
}
