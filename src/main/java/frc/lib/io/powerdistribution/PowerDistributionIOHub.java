/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.io.powerdistribution;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Device;

// Class for the Power Distribution Panel/Hub
public class PowerDistributionIOHub extends SubsystemBase implements PowerDistributionIO {
    
    // Create a Power Distribution object
    PowerDistribution powerDistribution;
    ModuleType type;
    
    // Create Alert objects for the Driver Station
    Alert voltageAlert;
    Alert currentAlert;
    Alert temperatureAlert;

    // Create inputs
    PowerDistributionInputsAutoLogged ioInputs = new PowerDistributionInputsAutoLogged();

    /** Constructor for the PowerDistributionIOHub.
     * 
     * @param id the CAN ID of the Power Distribution Hub/Panel.
     * @param name the human friendly name of the Power Distribution Hub/Panel, used for the Alert messages.
     * @param type choose kREV or kCTRE according to which vendor's Power Distribution Hub/Panel you are using.
     */
    public PowerDistributionIOHub(Device.CAN id, String name, ModuleType type) {
        this.type = type;
        powerDistribution = new PowerDistribution(id.id(), type);

        voltageAlert = new Alert(name, "Brownout! Voltage is below 8.0V!", Alert.AlertType.kWarning);
        currentAlert = new Alert(name, "Current exceeds 120A!", Alert.AlertType.kWarning);
        temperatureAlert = new Alert(name, "Temperature exceeds 70C!", Alert.AlertType.kWarning);
    }

    @Override
    public void updateInputs(PowerDistributionInputs inputs) {

        inputs.voltage = powerDistribution.getVoltage();
        inputs.currents = powerDistribution.getAllCurrents();
        inputs.totalCurrent = powerDistribution.getTotalCurrent();

        /* Inputs that are only supported on the CTRE Power Distribution Panel */
        if (type == ModuleType.kCTRE) {
            inputs.temperature = powerDistribution.getTemperature();
            inputs.totalPower = powerDistribution.getTotalPower();
            inputs.totalEnergy = powerDistribution.getTotalEnergy();
        }

        if (inputs.voltage < 8.0) {
            voltageAlert.set(true);
        } else {
            voltageAlert.set(false);
        }

        if (inputs.totalCurrent > 120.0) {
            currentAlert.set(true);
        } else {
            currentAlert.set(false);
        }

        if (inputs.temperature > 70.0) {
            temperatureAlert.set(true);
        } else {
            temperatureAlert.set(false);
        }
    }

    @Override
    public void periodic() {
        updateInputs(ioInputs);
    }
}