package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public class IntoTheDeepAppendageCommand {

    public final AppendageControlState AppendageState;
    public final Boolean Dunk;
    public final Boolean PincerOpen;
    public final Double CollectDistanceSignal;
    public final Double CollectHeightSignal;
    public final Double WristSignal;

    public IntoTheDeepAppendageCommand(
            @NonNull AppendageControlState appendageState,
            @NonNull Boolean dunk,
            @NonNull Boolean pincerOpen,
            @Nullable Double collectDistanceSignal,
            @Nullable Double collectHeightSignal,
            @Nullable Double wristSignal) {
        AppendageState = appendageState;
        Dunk = dunk;
        PincerOpen = pincerOpen;
        CollectDistanceSignal = collectDistanceSignal;
        CollectHeightSignal = collectHeightSignal;
        WristSignal = wristSignal;
    }

    public IntoTheDeepAppendageCommand withDunk(boolean dunk) {
        return new IntoTheDeepAppendageCommand(
                AppendageState,
                dunk,
                PincerOpen,
                CollectDistanceSignal,
                CollectHeightSignal,
                WristSignal);
    }

    public IntoTheDeepAppendageCommand withPincerOpen(boolean pincerOpen) {
        return new IntoTheDeepAppendageCommand(
                AppendageState,
                Dunk,
                pincerOpen,
                CollectDistanceSignal,
                CollectHeightSignal,
                WristSignal);
    }

    public static IntoTheDeepAppendageCommand highBasketPre() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.HIGH_BASKET,
                false,
                false,
                null,
                null,
                null);
    }

    public static IntoTheDeepAppendageCommand highBasketDunk() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.HIGH_BASKET,
                true,
                false,
                null,
                null,
                null);
    }

    public static IntoTheDeepAppendageCommand highBasketDunkRelease() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.HIGH_BASKET,
                true,
                true,
                null,
                null,
                null);
    }

    public static IntoTheDeepAppendageCommand collectingOpenAligned(double collectDistanceSignal, double collectHeightSignal) {
        return collectingOpen(collectDistanceSignal, collectHeightSignal, 0d);
    }

    public static IntoTheDeepAppendageCommand collectingClosedAligned(double collectDistanceSignal, double collectHeightSignal) {
        return collectingClosed(collectDistanceSignal, collectHeightSignal, 0d);
    }

    public static IntoTheDeepAppendageCommand collectingOpenAcross(double collectDistanceSignal, double collectHeightSignal) {
        return collectingOpen(collectDistanceSignal, collectHeightSignal, 0.9d);
    }

    public static IntoTheDeepAppendageCommand collectingClosedAcross(double collectDistanceSignal, double collectHeightSignal) {
        return collectingClosed(collectDistanceSignal, collectHeightSignal, 0.9d);
    }

    public static IntoTheDeepAppendageCommand collectingOpen(double collectDistanceSignal, double collectHeightSignal, double wristSignal) {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.COLLECTING,
                false,
                true,
                collectDistanceSignal,
                collectHeightSignal,
                wristSignal);
    }

    public static IntoTheDeepAppendageCommand collectingClosed(double collectDistanceSignal, double collectHeightSignal, double wristSignal) {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.COLLECTING,
                false,
                false,
                collectDistanceSignal,
                collectHeightSignal,
                wristSignal);
    }

    public static IntoTheDeepAppendageCommand defensive() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.DEFENSIVE,
                false,
                true,
                null,
                null,
                null);
    }
}
