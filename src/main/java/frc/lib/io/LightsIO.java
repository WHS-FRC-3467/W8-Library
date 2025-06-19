package frc.lib.io;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

public abstract class LightsIO {
  private LEDSegment[] segments;

  public LightsIO(LEDSegment[] segments) {
    this.segments = segments;
  }

      class LEDSegment {

        int startIndex;
        int segmentSize;
        int animationSlot;

        public LEDSegment(int startIndex, int segmentSize, int animationSlot)
        {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }
    }


}
