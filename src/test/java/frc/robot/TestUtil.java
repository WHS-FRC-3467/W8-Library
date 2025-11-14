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

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestUtil {
      
    private static Timer timer = new Timer();
        
    public static void runTest(Command command, double duration, Subsystem subsystem) {
        command.initialize();
        timer.restart();
        while (timer.get() < duration) {
            command.execute();
            subsystem.periodic();
            Timer.delay(0.02);
        }
        subsystem.periodic();
        timer.stop();
        command.end(true);
    }
}
