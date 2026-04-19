package frc.robot.energy;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class BatteryLogger {
    private double totalCurrent;
    private double driveCurrent;
    private double totalPower;
    private double totalEnergy;

    private double batteryVoltage;
    private double rioCurrent;
    private double frameWorkCurrent;

    private Map<String, Double> subsystemCurrents;
    private Map<String, Double> subsystemPowers;
    private Map<String, Double> subsystemEnergies;

    public void reportCurrentData(String key, boolean drive, double... amps) {
        double current = 0;
        for (double amp : amps) {
            current += Math.abs(amp);
        }
        subsystemCurrents.put(key, current);
        totalCurrent += current;
        if (drive) {
            driveCurrent += current;
        }
        double power = batteryVoltage * current;
        double energy = power * 0.02; // Assuming this method is called every 20ms

        subsystemPowers.put(key, power);
        subsystemEnergies.put(key, energy);
        subsystemCurrents.put(key, current);

        String[] keys = key.split("/|-");

        if (keys.length < 1) {
            return;
        }
        ;

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsystemCurrents.merge(subkey, totalCurrent, Double::sum);
            subsystemPowers.merge(subkey, totalPower, Double::sum);
            subsystemEnergies.merge(subkey, totalEnergy, Double::sum);
        }
    }

    public void periodicAfterScheduler() {
        reportCurrentData("Controls/roboRIO", false, rioCurrent);
        reportCurrentData("Controls/CANcoders", false, null);
        reportCurrentData("Controls/CANandgyro", false, null);
        reportCurrentData("Controls/Radio", false, 0.5);

        Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
        Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
        Logger.recordOutput("EnergyLogger/Energy", totalEnergy, "watt hours");

        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsystemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsystemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput(
                    "EnergyLogger/Energy/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()),
                    "watt hours");
        }
    }

    public void resetTotals() {
        // Reset power and curren totals, before next loop
        totalPower = 0.0;
        totalCurrent = 0.0;
        driveCurrent = 0.0;
    }

    public double getTotalCurrent() {
        return totalCurrent;
    }

    public double getTotalPower() {
        return totalPower;
    }

    public double getTotalEnergy() {
        return totalEnergy;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}
