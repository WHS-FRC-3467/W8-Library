// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib.io.LightsIO;
import frc.lib.io.LightsIOCandle;
import frc.lib.io.LightsIOCandle.LightsIOCandleConfiguration;
import frc.lib.io.LightsIOSim;
import frc.robot.Ports;
import frc.robot.Robot;

public class LEDsConstants {

    public static final LEDSegment candleLEDs = new LEDSegment(0, 7, 0);
    public static final LEDSegment frontStrip = new LEDSegment(8, 10, 1);

    public static final LightsIOCandleConfiguration getLightsIOCandleConfig() {
        LightsIOCandleConfiguration config = new LightsIOCandleConfiguration();
        config.id = Ports.CANDLE.id;
        config.bus = Ports.CANDLE.bus;
        config.ledConfig.LED.BrightnessScalar = 1.0;
        config.ledConfig.LED.StripType = StripTypeValue.RGB;
        config.ledConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        config.candleConfig.VBatOutputMode = VBatOutputModeValue.On;
        config.candleConfig.Enable5VRail = Enable5VRailValue.Disabled;
        config.candleConfig.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        return config;
    }

    public static final LightsIO getLightsIO() {
        if (Robot.isReal()) {
            return new LightsIOCandle(getLightsIOCandleConfig());
        } else {
            return new LightsIOSim();
        }
    }

    public record LEDSegment(int startIndex, int endIndex, int animationSlot) {
    };

    // Animations

    // Off
    public static final ControlRequest candleOff = new EmptyAnimation(candleLEDs.animationSlot);

    public static final ControlRequest frontOff = new EmptyAnimation(frontStrip.animationSlot);;

    public static final ControlRequest[] offAnimation = { candleOff, frontOff };

    // Disabled
    public static final ControlRequest candleDisabled = new RainbowAnimation(candleLEDs.startIndex, candleLEDs.endIndex)
            .withSlot(candleLEDs.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward);

    public static final ControlRequest frontDisabled = new RainbowAnimation(frontStrip.startIndex, frontStrip.endIndex)
            .withSlot(frontStrip.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward);

    public static final ControlRequest[] disabled = { candleDisabled, frontDisabled };

    // Auto
    public static final ControlRequest candleAuto = new FireAnimation(candleLEDs.startIndex, candleLEDs.endIndex)
            .withSlot(candleLEDs.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward)
            .withCooling(0.1)
            .withSparking(1.0);

    public static final ControlRequest frontAuto = new FireAnimation(frontStrip.startIndex, frontStrip.endIndex)
            .withSlot(frontStrip.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward)
            .withCooling(0.1)
            .withSparking(1.0);

    public static final ControlRequest[] autoAnimation = { candleAuto, frontAuto };

    // Flashing
    public static final ControlRequest candleFlash = new StrobeAnimation(candleLEDs.startIndex, candleLEDs.endIndex)
            .withSlot(candleLEDs.animationSlot)
            .withFrameRate(10)
            .withColor(new RGBWColor(Color.kRed));

}
