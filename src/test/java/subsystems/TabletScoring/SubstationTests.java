package subsystems.TabletScoring;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.TabletScoring.Substation;

public class SubstationTests {
    @Test
    void IsDoubleSubstation() {
        assertEquals(true, Substation.DOUBLE_LEFT.isDoubleSubstation());
        assertEquals(true, Substation.DOUBLE_RIGHT.isDoubleSubstation());

        assertEquals(false, Substation.NONE.isDoubleSubstation());
        assertEquals(false, Substation.SINGLE.isDoubleSubstation());
    }

    @Test
    void IsSingleSubstation() {
        assertEquals(true, Substation.SINGLE.isSingleSubstation());

        assertEquals(false, Substation.NONE.isSingleSubstation());
        assertEquals(false, Substation.DOUBLE_LEFT.isSingleSubstation());
        assertEquals(false, Substation.DOUBLE_RIGHT.isSingleSubstation());
    }
}
