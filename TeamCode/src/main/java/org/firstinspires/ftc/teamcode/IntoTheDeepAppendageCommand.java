package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public class IntoTheDeepAppendageCommand {

    public final AppendageControlState AppendageState;
    public final Boolean Dunk;
    public final Boolean PincerOpen;
    public final Double CollectDistanceInches;
    public final Double CollectHeightInches;
    public final Double CollectDistanceSignal;
    public final Double CollectHeightSignal;

    public IntoTheDeepAppendageCommand(
            @NonNull AppendageControlState appendageState,
            @NonNull Boolean dunk,
            @NonNull Boolean pincerOpen,
            @Nullable Double collectDistanceInches,
            @Nullable Double collectHeightInches,
            @Nullable Double collectDistanceSignal,
            @Nullable Double collectHeightSignal) {
        AppendageState = appendageState;
        Dunk = dunk;
        PincerOpen = pincerOpen;
        CollectDistanceInches = collectDistanceInches;
        CollectHeightInches = collectHeightInches;
        CollectDistanceSignal = collectDistanceSignal;
        CollectHeightSignal = collectHeightSignal;
    }

    public IntoTheDeepAppendageCommand withDunk(boolean dunk) {
        return new IntoTheDeepAppendageCommand(
                AppendageState,
                dunk,
                PincerOpen,
                CollectDistanceInches,
                CollectHeightInches,
                CollectDistanceSignal,
                CollectHeightSignal);
    }

    public IntoTheDeepAppendageCommand withPincerOpen(boolean pincerOpen) {
        return new IntoTheDeepAppendageCommand(
                AppendageState,
                Dunk,
                pincerOpen,
                CollectDistanceInches,
                CollectHeightInches,
                CollectDistanceSignal,
                CollectHeightSignal);
    }

    public static IntoTheDeepAppendageCommand highBasketPre() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.HIGH_BASKET,
                false,
                false,
                null,
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
                null,
                null);
    }

    public static IntoTheDeepAppendageCommand collecting() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.COLLECTING,
                false,
                true,
                null,
                null,
                null,
                null);
    }

    public static IntoTheDeepAppendageCommand defensive() {
        return new IntoTheDeepAppendageCommand(
                AppendageControlState.DEFENSIVE,
                false,
                true,
                null,
                null,
                null,
                null);
    }
}
