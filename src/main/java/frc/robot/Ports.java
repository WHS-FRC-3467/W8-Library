package frc.robot;

import frc.lib.util.Device;
import frc.lib.util.Device.CAN;
import frc.lib.util.Device.DIO;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    public static final Device.CAN laserCAN1 = new CAN(0, "rio");
    public static final Device.CAN lights = new CAN(1, "rio");
    public static final Device.DIO diobeambreak = new DIO(2);
}
