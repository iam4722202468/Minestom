package net.minestom.server.entity.pathfinding;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.collision.PhysicsResult;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.Chunk;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Set;

public class PNode {
    public enum NodeType {
        WALK,
        JUMP,
        FALL,
        REPATH
    }

    double g;
    double h;
    PNode parent;
    Pos point;
    final int hashCode;

    private PNode tempNode = null;

    private NodeType type = NodeType.WALK;

    void setType(NodeType newType) {
        this.type = newType;
    }

    public NodeType getType() {
        return type;
    }

    public PNode(Pos point, double g, double h, PNode parent) {
        this.point = new Pos(point.x(), point.y(), point.z());
        this.g = g;
        this.h = h;
        this.parent = parent;
        this.hashCode = point.hashCode();
    }

    public PNode(Pos point, double g, double h, NodeType type, PNode parent) {
        this.point = new Pos(point.x(), point.y(), point.z());
        this.g = g;
        this.h = h;
        this.parent = parent;
        this.hashCode = point.hashCode();
        this.type = type;
    }

    @Override
    public int hashCode() {
        return hashCode;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) return false;
        if (obj == this) return true;
        if (!(obj instanceof PNode other)) return false;
        return this.point.samePoint(other.point);
    }

    public Collection<? extends PNode> getNearby(Instance instance, Set<PNode> closed, Point goal, @NotNull BoundingBox boundingBox) {
        Collection<PNode> nearby = new ArrayList<>();
        tempNode = new PNode(Pos.ZERO, 0, 0, this);

        int stepSize = (int) Math.max(Math.floor(boundingBox.width() / 2), 1);
        if (stepSize < 1) stepSize = 1;

        for (int x = -stepSize; x <= stepSize; ++x) {
            for (int z = -stepSize; z <= stepSize; ++z) {
                if (x == 0 && z == 0) continue;
                double cost = Math.sqrt(x * x + z * z) * 0.98;

                Pos floorPoint = point.withX(point.blockX() + 0.5 + x).withZ(point.blockZ() + 0.5 + z);
                Pos jumpPoint = point.withX(point.blockX() + 0.5 + x).withZ(point.blockZ() + 0.5 + z).add(0, 1, 0);

                floorPoint = gravitySnap(instance, floorPoint, boundingBox, 20);
                jumpPoint = gravitySnap(instance, jumpPoint, boundingBox, 20);

                if (floorPoint == null) continue;

                var nodeWalk = createWalk(instance, floorPoint, boundingBox, cost, point, goal, closed);
                if (nodeWalk != null && !closed.contains(nodeWalk)) nearby.add(nodeWalk);

                if (jumpPoint == null) continue;
                if (!floorPoint.sameBlock(jumpPoint)) {
                    var nodeJump = createJump(instance, jumpPoint, boundingBox, cost, point, goal, closed);
                    if (nodeJump != null && !closed.contains(nodeJump)) nearby.add(nodeJump);
                }
            }
        }

        return nearby;
    }

    private PNode createWalk(Instance instance, Pos point, BoundingBox boundingBox, double cost, Pos start, Point goal, Set<PNode> closed) {
        var n = newNode(cost, point, goal);
        if (closed.contains(n)) return null;

        if (point.y() < start.y()) {
            if (!canMoveTowards(instance, start, point.withY(start.y()), boundingBox)) return null;
            n.setType(NodeType.FALL);
        } else {
            if (!canMoveTowards(instance, start, point, boundingBox)) return null;
        }
        return n;
    }

    private PNode createJump(Instance instance, Pos point, BoundingBox boundingBox, double cost, Pos start, Point goal, Set<PNode> closed) {
        if (point.y() - start.y() == 0) return null;
        if (point.y() - start.y() > 2) return null;

        if (pointInvalid(instance, point, boundingBox)) return null;
        if (pointInvalid(instance, start.add(0, 1, 0), boundingBox)) return null;

        var n = newNode(cost, point, goal);
        n.setType(NodeType.JUMP);
        return n;
    }

    private boolean pointInvalid(Instance instance, Pos point, BoundingBox boundingBox) {
        var iterator = boundingBox.getBlocks(point);
        while (iterator.hasNext()) {
            var block = iterator.next();
            if (instance.getBlock(block, Block.Getter.Condition.TYPE).isSolid()) {
                return true;
            }
        }

        return false;
    }

    private PNode newNode(double cost, Pos point, Point goal) {
        tempNode.g = g + cost;
        tempNode.h = PathGenerator.heuristic(point, goal);
        tempNode.point = point;

        var newNode = tempNode;
        tempNode = new PNode(Pos.ZERO, 0, 0, this);

        return newNode;
    }

    static Pos gravitySnap(Instance instance, Point point, BoundingBox boundingBox, double maxFall) {
        Chunk c = instance.getChunkAt(point);
        if (c == null) return null;

        for (int axis = 1; axis <= maxFall; ++axis) {
            var iterator = boundingBox.getBlocks(point, BoundingBox.AxisMask.Y, -axis);

            while (iterator.hasNext()) {
                var block = iterator.next();
                if (instance.getBlock(block, Block.Getter.Condition.TYPE).isSolid()) {
                    return Pos.fromPoint(point.withY(Math.floor(point.y() - axis + 1)));
                }
            }
        }

        return Pos.fromPoint(point.withY(point.y() - maxFall));
    }

    private static boolean canMoveTowards(Instance instance, Pos start, Point end, BoundingBox boundingBox) {
        Point diff = end.sub(start);
        PhysicsResult res = CollisionUtils.handlePhysics(instance, instance.getChunkAt(start), boundingBox, start, Vec.fromPoint(diff), null, false);
        // res = CollisionUtils.handlePhysics(instance, instance.getChunkAt(start), boundingBox, res.newPosition(), Vec.fromPoint(diff), res, false);
        return !res.collisionZ() && !res.collisionY() && !res.collisionX();
    }

    @Override
    public String toString() {
        return "PNode{" +
                "point=" + point +
                ", d=" + (g + h) +
                ", type=" + type +
                '}';
    }

    public Point point() {
        return point;
    }
}
