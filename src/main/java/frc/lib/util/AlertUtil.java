/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.util;

import org.wpilib.util.Alert;

import java.util.HashMap;
import java.util.Map;

/**
 * Helpers for WPILib 2027 {@link Alert}s.
 *
 * <p>In WPILib 2027, every alert needs an id that is unique within its group, and constructing a
 * second alert with an id that is already allocated throws. Robot code (and especially unit tests,
 * which construct subsystems repeatedly) can create alerts with the same id more than once, so this
 * hands out a unique id by appending a suffix on repeats.
 */
public final class AlertUtil {
    private static final Map<String, Integer> idCounts = new HashMap<>();

    private AlertUtil() {}

    /**
     * Creates an alert in the default group, making the id unique if it has been used before.
     *
     * @param id Preferred alert identifier
     * @param text Text to be displayed when the alert is active
     * @param level Alert urgency level
     * @return The new alert
     */
    public static synchronized Alert create(String id, String text, Alert.Level level) {
        int count = idCounts.merge(id, 1, Integer::sum);
        return new Alert(count == 1 ? id : id + " (" + count + ")", text, level);
    }
}
