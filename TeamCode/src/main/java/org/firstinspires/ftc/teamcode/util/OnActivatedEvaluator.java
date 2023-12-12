package org.firstinspires.ftc.teamcode.util;

public class OnActivatedEvaluator {
    private final BooleanStateReader booleanStateReader;
    private boolean wasActivated;

    public OnActivatedEvaluator(BooleanStateReader buttonStateReader) {
        this.booleanStateReader = buttonStateReader;
        this.wasActivated = false;
    }

    public boolean evaluate() {
        boolean currentlyPressed = booleanStateReader.isActivated();
        if (currentlyPressed) {
            if (!wasActivated) {
                wasActivated = true;
                return true;
            }
        } else {
            wasActivated = false;
        }
        return false;
    }
}

