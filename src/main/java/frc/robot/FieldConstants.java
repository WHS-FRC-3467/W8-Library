// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final double FIELDLENGTH = Units.inchesToMeters(690.876);
    public static final double FIELDWIDTH = Units.inchesToMeters(317);
    public static final Translation2d fieldCenter =
        new Translation2d(FIELDLENGTH / 2, FIELDWIDTH / 2);
    public static final double startingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting

    public static final Distance ALGAEDIAMETER = Meters.of(.41);

}
