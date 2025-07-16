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

package frc.lib.io.servo;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;

public class ServoIOSim implements ServoIO {
    
    @Getter
    private final String name;
    private final double lowerLimitDegrees;
    private final double upperLimitDegrees;

    private final LoggedTunableNumber targetPositionDegrees;

    /**
     * Constructs a {@link ServoIOSim} object with the specified name and limits.
     *
     * @param name A human-readable name for this servo instance
     * @param lowerLimitDegrees The lower limit of the servo in degrees.
     * @param upperLimitDegrees The upper limit of the servo in degrees.
     */
    public ServoIOSim(String name, double lowerLimitDegrees, double upperLimitDegrees) {
        this.name = name;
        this.targetPositionDegrees = new LoggedTunableNumber(name, 0);
        this.lowerLimitDegrees = lowerLimitDegrees;
        this.upperLimitDegrees = upperLimitDegrees;
    }

    @Override
    public void updateInputs(ServoInputs inputs) {
        inputs.position = Degrees.of(MathUtil.clamp(targetPositionDegrees.getAsDouble(), lowerLimitDegrees, upperLimitDegrees));
    }
}
