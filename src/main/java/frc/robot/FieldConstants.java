// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Distance FIELDLENGTH = Meters.of(aprilTagLayout.getFieldLength());
    public static final Distance FIELDWIDTH = Meters.of(aprilTagLayout.getFieldWidth());

    public static final Distance STARTINGLINEX = Inches.of(299.438);

    public static final Translation2d FIELDCENTER =
        new Translation2d(FIELDLENGTH.in(Meters) / 2, FIELDWIDTH.in(Meters) / 2);

    public static final Distance ALGAEDIAMETER = Meters.of(.41);

}
