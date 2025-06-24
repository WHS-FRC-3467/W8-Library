package frc.robot;

import frc.lib.util.Device;
import frc.lib.util.Device.CAN;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    public static final Device.CAN laserCAN1 = new CAN(0, "rio");
    public static final Device.CAN lights = new CAN(1, "rio");

}
