package frc.lib.devices.apriltagcameras;

import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public interface AprilTagCamera {
    void periodic();

    public static record CameraProperties(
        String name,
        Transform3d robotToCamera,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs,
        int resolutionWidth,
        int resolutionHeight,
        double stdDevFactor) {
    }

    public static record TagObservation(
        int id,
        double area,
        Angle pitch,
        Angle yaw,
        List<TargetCorner> targetCorners,
        Transform3d cameraToTarget,
        double ambiguity,
        Distance distance) {
        public TagObservation(
            int id,
            double area,
            Angle pitch,
            Angle yaw,
            List<TargetCorner> targetCorners,
            Transform3d cameraToTarget,
            double ambiguity)
        {
            this(
                id,
                area,
                pitch,
                yaw,
                targetCorners,
                cameraToTarget,
                ambiguity,
                Meters.of(cameraToTarget.getTranslation().getNorm()));
        }

        public static record VisionObservation(
            Time timestamp,
            CameraProperties camera,
            Optional<Pose3d> multiTagPose,
            List<TagObservation> tagObservations) {
        }
    }
}
