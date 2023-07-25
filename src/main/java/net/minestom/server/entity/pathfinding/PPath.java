package net.minestom.server.entity.pathfinding;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.Instance;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class PPath {
    private final Consumer<Void> onComplete;
    private final BoundingBox boundingBox;
    private final Instance instance;
    private final List<PNode> nodes = new ArrayList<>();
    private final double pathVariance;
    private final double maxDistance;
    private int index = 0;
    private final Pos initialPosition;
    private final AtomicReference<PathState> state = new AtomicReference<>(PathState.COMPUTING);

    enum PathState {
        COMPUTING,
        FOLLOWING,
        COMPLETED
    }

    PathState getState() {
        return state.get();
    }

    public List<PNode> getNodes() {
        return nodes;
    }

    public PPath(Pos point, Instance instance, BoundingBox boundingBox, double maxDistance, double pathVariance, Consumer<Void> onComplete) {
        this.onComplete = onComplete;
        this.initialPosition = point;
        this.instance = instance;
        this.boundingBox = boundingBox;
        this.maxDistance = maxDistance;
        this.pathVariance = pathVariance;
    }

    void runComplete() {
        if (onComplete != null) onComplete.accept(null);
    }

    @Override
    public String toString() {
        return nodes.toString();
    }

    PNode.NodeType getCurrentType() {
        if (index >= nodes.size()) return null;
        var current = nodes.get(index);
        return current.getType();
    }

    @Nullable
    Point getCurrent() {
        if (index >= nodes.size()) return null;
        var current = nodes.get(index);
        return current.point;
    }

    void next() {
        if (index >= nodes.size()) return;
        index++;
    }

    double maxDistance() {
        return maxDistance;
    }

    double pathVariance() {
        return pathVariance;
    }
}
