package frc.util.field;

import java.util.ArrayList;

public class MirroredFieldZone {
    private ArrayList<MirroredSubzone> mirroredSubzones = new ArrayList<MirroredSubzone>();
    private FieldZone blueFieldZone;
    private FieldZone redFieldZone;

    public MirroredFieldZone(String zoneName, MirroredSubzone initialMirroredSubzone) {
        blueFieldZone = new FieldZone("Blue " + zoneName, initialMirroredSubzone.getBlueSubzone());
        initialMirroredSubzone.getBlueSubzone().setFieldZone(blueFieldZone);

        redFieldZone = new FieldZone("Red " + zoneName, initialMirroredSubzone.getRedSubzone());
        initialMirroredSubzone.getRedSubzone().setFieldZone(redFieldZone);
    }

    // Create a field zone, which must be initialized with at least one subzone.
    public void addMirroredSubzone(MirroredSubzone mirroredSubzone) {
        mirroredSubzones.add(mirroredSubzone);
        
        blueFieldZone.addSubzone(mirroredSubzone.getBlueSubzone());
        mirroredSubzone.getBlueSubzone().setFieldZone(blueFieldZone);

        redFieldZone.addSubzone(mirroredSubzone.getRedSubzone());
        mirroredSubzone.getRedSubzone().setFieldZone(redFieldZone);
    }

    public FieldZone getBlueFieldZone() {
        return blueFieldZone;
    }

    public FieldZone getRedFieldZone() {
        return redFieldZone;
    }
}
