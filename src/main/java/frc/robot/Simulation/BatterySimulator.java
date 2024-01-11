package frc.robot.Simulation;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class BatterySimulator {

    private final ArrayList<Supplier<Double>> currentDraws;

    private static BatterySimulator batterySim;

    private BatterySimulator() {
        currentDraws = new ArrayList<>();
        currentDraws.add(() -> 0.0);
    }

    public static BatterySimulator getInstance() {
        if(batterySim == null) {
            batterySim = new BatterySimulator();
        }

        return batterySim;
    }

    /**
     * Adds a source of current draw for a simulated battery. Call for every device.
     * @param currentSupplier Lambda function that supplies the current draw amount from a particular device
     */
    public void addCurrentDrawSource(Supplier<Double> currentSupplier) {
        this.currentDraws.add(currentSupplier);
    }

    /**
     * Function that is called periodically to update the simulated Battery's Voltage.
     * Voltage value is accessed through RobotController.getBatteryVoltage() as normal.
     */
    public void updateSimulatedBatteryVoltage() {
        double currentDrawSum = 0;

        for(int i=0; i<currentDraws.size(); i++) {
            currentDrawSum += currentDraws.get(i).get();
        }

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currentDrawSum));
    }
    
}
