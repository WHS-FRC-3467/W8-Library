// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.CANDevice;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class DistanceIOLaserCAN implements DistanceIO {
    private final LaserCanInterface laserCan;
    private final String name;
    private Distance currentDistance;
    private double tries = 0;
    private boolean hasConfiged = false;

    private final Alert failedConfig = new Alert("Failed to configure LaserCAN!", AlertType.kError);
    private final Alert sensorAlert =
        new Alert("Failed to get LaserCAN measurement", Alert.AlertType.kWarning);

    public DistanceIOLaserCAN(
        CANDevice id,
        String name,
        RangingMode rangingMode,
        RegionOfInterest regionOfInterest,
        TimingBudget timingBudget)
    {
        this.name = name;
        this.laserCan =
            (Constants.currentMode == Constants.simMode)
                ? new LaserCANSim(name)
                : new LaserCan(id.id());
        while (tries < 5) {
            try {
                this.laserCan.setRangingMode(rangingMode);
                this.laserCan.setRegionOfInterest(regionOfInterest);
                this.laserCan.setTimingBudget(timingBudget);
                failedConfig.set(false);
                System.out.println("Succesfully configured " + name);
                hasConfiged = true;
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed for " + name + "! " + e);
                failedConfig.setText("Failed to configure " + name + "!");
                failedConfig.set(true);
                tries++;
            }
        }
    }

    /**
     * Updates the DistanceIO inputs
     *
     * @param inputs The DistanceIOInputs object where the updated values will be stored.
     */
    @Override
    public void updateInputs(DistanceIOInputs inputs)
    {
        Measurement measurement = laserCan.getMeasurement();
        if (measurement != null) {
            if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                sensorAlert.set(false);
                currentDistance = Millimeters.of(measurement.distance_mm);
                inputs.ambientSignal = (double) measurement.ambient;
            } else {
                sensorAlert
                    .setText("Failed to get LaserCAN ID: " + name + ", no valid measurement");
                sensorAlert.set(true);
                currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
                inputs.connected = false;
            }
        } else {
            sensorAlert.setText("Failed to get LaserCAN ID: " + name + ", measurement null");
            sensorAlert.set(true);
            currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
        }
        Logger.recordOutput("LaserCANSensors/LaserCAN" + name, currentDistance.in(Inches));
        inputs.distance = currentDistance;
    }

    /**
     * Updates the current settings that determine whether the CANrange 'detects' an object. Call
     * this method to achieve the desired performance.
     *
     * @param mode The LONG Ranging Mode can be used to identify targets at longer distances than
     *        the short ranging mode (up to 4m), but is more susceptible to ambient light. The SHORT
     *        Ranging Mode is used to detect targets at 1.3m and lower. Although shorter than the
     *        Long ranging mode, this mode is less susceptible to ambient light.
     * @throws ConfigurationFailedException
     */
    private void setRangingMode(RangingMode mode)
    {
        tries = 0;
        while (tries < 5) {
            try {
                laserCan.setRangingMode(mode);
                hasConfiged = true;
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed for " + name + "! " + e);
                failedConfig.setText("Failed to configure " + name + " new ranging mode!");
                failedConfig.set(true);
                tries++;
            }
        }
    }

    /**
     * Updates the current settings that determine whether the center and extent of the CANrange's
     * view Call this method to achieve the desired performance.
     *
     * @see parameters in the ROI configs: newROICenterX: Specifies the target center of the Field
     *      of View in the X direction, between +/- 11.8
     * 
     * @see newROICenterY: Specifies the target center of the Field of View in the Y direction,
     *      between +/- 11.8
     * @see newROIRangeX: Specifies the target range of the Field of View in the X direction. The
     *      magnitude of this is capped to abs(27 - 2*FOVCenterY).
     * @see newROIRangeY: Specifies the target range of the Field of View in the Y direction. The
     *      magnitude of this is capped to abs(27 - 2*FOVCenterY).
     * @param roiConfig The Region of Interest configuration to set, including the center and range
     *        in
     * @throws ConfigurationFailedException
     */
    private void setROIParams(RegionOfInterest roiConfig)
    {
        hasConfiged = false;
        tries = 0;
        while (!hasConfiged && tries < 5) {
            try {
                laserCan.setRegionOfInterest(roiConfig);
                hasConfiged = true;
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed for " + name + "! " + e);
                failedConfig.setText("Failed to configure " + name + " new ROI params!");
                failedConfig.set(true);
                tries++;
            }
        }
    }

    /**
     * Returns the current distance measured by the Distance sensor in meters
     *
     * @return The current distance as a Distance object.
     */
    @Override
    public Distance getDistance()
    {
        return currentDistance;
    }

    /**
     * Checks if the measured distance is within a specified tolerance of an expected distance.
     *
     * @param expected The expected distance to compare against.
     * @param tolerDistance The tolerance distance within which the measurement is considered
     *        "near".
     * @return true if the measured distance is within the tolerance of the expected distance, false
     *         otherwise.
     */
    @Override
    public boolean isNearDistance(Distance expected, Distance tolerDistance)
    {
        return MathUtil.isNear(
            expected.in(Millimeters), getDistance().in(Millimeters), tolerDistance.in(Millimeters));
    }
}
