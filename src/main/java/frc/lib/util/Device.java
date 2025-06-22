package frc.lib.util;

public sealed interface Device {
    record CAN(int id, String bus) implements Device {
    }

    public record DIO(int id) implements Device {
    }
}
