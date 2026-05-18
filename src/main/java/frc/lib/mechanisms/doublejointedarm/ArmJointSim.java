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
package frc.lib.mechanisms.doublejointedarm;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.lib.io.motor.MotorIO.ControlType;

import lombok.Setter;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class ArmJointSim extends SingleJointedArmSim {

    private final double minAngleRads;
    private final double maxAngleRads;
    private final boolean simulateGravity;
    private final double armLengthMeters;

    private @Setter Optional<AngularVelocity> attachedVelocity = Optional.empty();
    private @Setter Optional<Double> attachedAngle = Optional.empty();

    @Setter private ControlType controlType;

    public ArmJointSim(
            DCMotor gearbox,
            double gearing,
            double jKgMetersSquared,
            double armLengthMeters,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            double startingAngleRads,
            double... measurementStdDevs) {
        super(
                gearbox,
                gearing,
                jKgMetersSquared,
                armLengthMeters,
                minAngleRads,
                maxAngleRads,
                simulateGravity,
                startingAngleRads,
                measurementStdDevs);

        this.minAngleRads = minAngleRads;
        this.maxAngleRads = maxAngleRads;
        this.simulateGravity = simulateGravity;
        this.armLengthMeters = armLengthMeters;
    }

    private double calculateFriction(double velocity) {
        double friction = velocity;

        if (m_x.get(0, 0) >= maxAngleRads - 0.01) {
            return -0.01;

        } else if (m_x.get(0, 0) <= minAngleRads + 0.01) {
            return 0.01;
        }

        if (velocity > 0.0) {
            if (velocity - 0.025 > 0.0) {
                friction = velocity - 0.025;
            } else {
                friction = velocity / 2;
            }
        } else {
            if (velocity + 0.025 < 0.0) {
                friction = velocity + 0.025;
            } else {
                friction = velocity / 2;
            }
        }
        return friction;
    }

    @Override
    public void update(double dtSeconds) {
        super.update(dtSeconds);

        m_y.set(1, 0, calculateFriction(m_y.get(1, 0)));
    }

    @Override
    protected Matrix<N2, N1> updateX(
            Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {

        Matrix<N2, N1> updatedXhat =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
                            Matrix<N2, N1> xdot =
                                    m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                            if (simulateGravity) {
                                double alphaGrav =
                                        3.0
                                                / 2.0
                                                * -9.8
                                                * Math.cos(
                                                        (attachedAngle.isPresent()
                                                                ? x.get(0, 0) + attachedAngle.get()
                                                                : x.get(0, 0)))
                                                / armLengthMeters;
                                if (attachedAngle.isPresent()) {
                                    Logger.recordOutput("alphaGrav", alphaGrav);
                                    Logger.recordOutput(
                                            "xabs",
                                            Math.abs(x.get(0, 0))
                                                    % Math.PI
                                                    * Math.signum(x.get(0, 0)));

                                    Logger.recordOutput("attachedAngle", attachedAngle.get());
                                }

                                if (attachedVelocity.isPresent()) {

                                    alphaGrav -=
                                            (((attachedVelocity.get().in(RadiansPerSecond) / 6)));
                                }
                                xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                            }
                            return xdot;
                        },
                        currentXhat,
                        u,
                        dtSeconds);

        // We check for collision after updating xhat

        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {

            return VecBuilder.fill(minAngleRads, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxAngleRads, 0);
        }
        return updatedXhat;
    }
}
