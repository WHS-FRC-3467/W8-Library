/*
 * Copyright (C) 2025 Windham Windup
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


package frc.robot;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TestUtil {

    private static final boolean USE_TIMING = false;

    private static final int NOTIFIER = NotifierJNI.initializeNotifier();
    private static final CommandScheduler SCHEDULER = CommandScheduler.getInstance();
    private static final Timer TIMER = new Timer();

    public static void runTest(Command command, double duration)
    {
        SCHEDULER.cancelAll();

        command.schedule();

        TIMER.start();

        double nextCycleSeconds = 0.0;
        while (true) {
            if (USE_TIMING) {
                double currentTime = TIMER.get();
                if (nextCycleSeconds < currentTime) {
                    // Loop overrun, start next cycle immediately
                    nextCycleSeconds = currentTime;
                } else {
                    // Wait before next cycle
                    NotifierJNI.updateNotifierAlarm(NOTIFIER, (long) nextCycleSeconds * 1000000);
                    if (NotifierJNI.waitForNotifierAlarm(NOTIFIER) == 0L) {
                        // Break the loop if the notifier was stopped
                        break;
                    }
                }
                nextCycleSeconds += 0.02;
            }

            SCHEDULER.run();
            if (TIMER.hasElapsed(duration))
                break;
        }

        TIMER.stop();
        TIMER.reset();

        SCHEDULER.cancelAll();
    }
}
