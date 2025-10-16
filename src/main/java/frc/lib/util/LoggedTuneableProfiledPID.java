// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LoggedTuneableProfiledPID extends ProfiledPIDController {

    // Tunable numbers
    private LoggedTunableNumber m_kP, m_kI, m_kD, m_maxV, m_maxA;

    public LoggedTuneableProfiledPID(String name, double kP, double kI,
        double kD, double maxV, double maxA)
    {
        this(name, kP, kI, kD, maxV, maxA, .02);
    }

    public LoggedTuneableProfiledPID(String name, double kP, double kI,
        double kD, double maxV, double maxA, double period)
    {
        super(kP, kI, kD, new TrapezoidProfile.Constraints(maxV, maxA), period);

        // Tunable numbers for PID and motion gain constants
        m_kP = new LoggedTunableNumber(name + "/kP", kP);
        m_kI = new LoggedTunableNumber(name + "/kI", kI);
        m_kD = new LoggedTunableNumber(name + "/kD", kD);

        m_maxV = new LoggedTunableNumber(name + "/maxV", maxV);
        m_maxA = new LoggedTunableNumber(name + "/maxA", maxA);
    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (m_kP.hasChanged(hashCode())
            || m_kI.hasChanged(hashCode())
            || m_kD.hasChanged(hashCode())) {
            this.setPID(m_kP.get(), m_kI.get(), m_kD.get());
        }

        if (m_maxV.hasChanged(hashCode())
            || m_maxA.hasChanged(hashCode())) {
            this.setConstraints(new TrapezoidProfile.Constraints(m_maxV.get(), m_maxA.get()));
        }
    }

}
