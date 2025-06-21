package frc.lib.util;

public sealed interface Device {
    record CAN(int id, String bus) implements Device {
    }
}
