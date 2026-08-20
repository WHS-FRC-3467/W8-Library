/*
 * Copyright (c) 2009-2025 FIRST and other WPILib contributors All rights reserved.
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {

        // Source - https://stackoverflow.com/a/34839209
        // Posted by TWiStErRob, modified by community. See post 'Timeline' for change history
        // Retrieved 2026-06-19, License - CC BY-SA 3.0

        System.setOut(
                new java.io.PrintStream(
                        new java.io.OutputStream() {
                            @Override
                            public void write(int b) {}
                        }) {
                    @Override
                    public void flush() {}

                    @Override
                    public void close() {}

                    @Override
                    public void write(int b) {}

                    @Override
                    public void write(byte[] b) {}

                    @Override
                    public void write(byte[] buf, int off, int len) {}

                    @Override
                    public void print(boolean b) {}

                    @Override
                    public void print(char c) {}

                    @Override
                    public void print(int i) {}

                    @Override
                    public void print(long l) {}

                    @Override
                    public void print(float f) {}

                    @Override
                    public void print(double d) {}

                    @Override
                    public void print(char[] s) {}

                    @Override
                    public void print(String s) {}

                    @Override
                    public void print(Object obj) {}

                    @Override
                    public void println() {}

                    @Override
                    public void println(boolean x) {}

                    @Override
                    public void println(char x) {}

                    @Override
                    public void println(int x) {}

                    @Override
                    public void println(long x) {}

                    @Override
                    public void println(float x) {}

                    @Override
                    public void println(double x) {}

                    @Override
                    public void println(char[] x) {}

                    @Override
                    public void println(String x) {}

                    @Override
                    public void println(Object x) {}

                    @Override
                    public java.io.PrintStream printf(String format, Object... args) {
                        return this;
                    }

                    @Override
                    public java.io.PrintStream printf(
                            java.util.Locale l, String format, Object... args) {
                        return this;
                    }

                    @Override
                    public java.io.PrintStream format(String format, Object... args) {
                        return this;
                    }

                    @Override
                    public java.io.PrintStream format(
                            java.util.Locale l, String format, Object... args) {
                        return this;
                    }

                    @Override
                    public java.io.PrintStream append(CharSequence csq) {
                        return this;
                    }

                    @Override
                    public java.io.PrintStream append(CharSequence csq, int start, int end) {
                        return this;
                    }

                    @Override
                    public java.io.PrintStream append(char c) {
                        return this;
                    }
                });

        RobotBase.startRobot(Robot::new);
    }
}
