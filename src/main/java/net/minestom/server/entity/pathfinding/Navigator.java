package net.minestom.server.entity.pathfinding;

import net.minestom.server.attribute.Attribute;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.instance.Chunk;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.WorldBorder;
import net.minestom.server.particle.Particle;
import net.minestom.server.particle.ParticleCreator;
import net.minestom.server.utils.chunk.ChunkUtils;
import net.minestom.server.utils.position.PositionUtils;
import org.jetbrains.annotations.ApiStatus;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;
import java.util.function.Consumer;

// TODO all pathfinding requests could be processed in another thread

/**
 * Necessary object for all {@link NavigableEntity}.
 */
public final class Navigator {
    private Point goalPosition;
    private final Entity entity;
    private PPath path;
    private double minimumDistance;
    private float movementSpeed = 0.1f;

    public Navigator(@NotNull Entity entity) {
        this.entity = entity;

        if (entity instanceof LivingEntity living) {
            movementSpeed = living.getAttribute(Attribute.MOVEMENT_SPEED).getBaseValue();
        }
    }

    public PPath.PathState getState() {
        return path.getState();
    }

    /**
     * Used to move the entity toward {@code direction} in the X and Z axis
     * Gravity is still applied but the entity will not attempt to jump
     * Also update the yaw/pitch of the entity to look along 'direction'
     *
     * @param direction    the targeted position
     * @param speed        define how far the entity will move
     * @param capabilities
     */
    public void moveTowards(@NotNull Point direction, double speed, PPath.PathfinderCapabilities capabilities) {
        final Pos position = entity.getPosition();
        final double dx = direction.x() - position.x();
        final double dy = direction.y() - position.y();
        final double dz = direction.z() - position.z();
        // the purpose of these few lines is to slow down entities when they reach their destination
        final double distSquared = dx * dx + dy * dy + dz * dz;
        if (speed > distSquared) {
            speed = distSquared;
        }

        boolean inWater = false;
        var instance = entity.getInstance();
        if (instance != null)
            if (instance.getBlock(position).isLiquid()) {
                speed *= capabilities.swimSpeedModifier();
                inWater = true;
            }

        final double radians = Math.atan2(dz, dx);
        final double speedX = Math.cos(radians) * speed;
        final double speedZ = Math.sin(radians) * speed;
        final float yaw = PositionUtils.getLookYaw(dx, dz);
        final float pitch = PositionUtils.getLookPitch(dx, dy, dz);

        final double speedY = (capabilities.type() == PPath.PathfinderType.AQUATIC
                || capabilities.type() == PPath.PathfinderType.FLYING
                || (capabilities.type() == PPath.PathfinderType.AMPHIBIOUS && inWater))
                ? Math.signum(dy) * 0.5 * speed
                : 0;

        // Prevent ghosting
        final var physicsResult = CollisionUtils.handlePhysics(entity, new Vec(speedX, speedY, speedZ));
        this.entity.refreshPosition(Pos.fromPoint(physicsResult.newPosition()).withView(yaw, pitch));
    }

    public void jump(float height) {
        // FIXME magic value
        this.entity.setVelocity(new Vec(0, height * 2.5f, 0));
    }

    public synchronized boolean setPathTo(@Nullable Point point) {
        BoundingBox bb = this.entity.getBoundingBox();
        double centerToCorner = Math.sqrt(bb.width() * bb.width() + bb.depth() * bb.depth()) / 2;
        return setPathTo(point, centerToCorner, null);
    }

    public synchronized boolean setPathTo(@Nullable Point point, double minimumDistance, Consumer<Void> onComplete) {
        return setPathTo(point, minimumDistance, 25, 5, PPath.PathfinderType.LAND, onComplete);
    }

    /**
     * Sets the path to {@code position} and ask the entity to follow the path.
     *
     * @param point the position to find the path to, null to reset the pathfinder
     * @param minimumDistance distance to target when completed
     * @param maxDistance maximum search distance
     * @param pathVariance how far to search off of the direct path. For open worlds, this can be low (around 20) and for large mazes this needs to be very high.
     * @param onComplete called when the path has been completed
     * @return true if a path has been found
     */
    public synchronized boolean setPathTo(@Nullable Point point, double minimumDistance, double maxDistance, double pathVariance, PPath.PathfinderType type, Consumer<Void> onComplete) {
        if (point != null && goalPosition != null && point.samePoint(goalPosition) && this.path != null) {
            // Tried to set path to the same target position
            return false;
        }

        if (this.path != null) {
            this.path.setState(PPath.PathState.TERMINATING);
        }

        final Instance instance = entity.getInstance();
        if (point == null) {
            this.path = null;
            return false;
        }

        // Can't path with a null instance.
        if (instance == null) {
            this.path = null;
            return false;
        }

        // Can't path outside the world border
        final WorldBorder worldBorder = instance.getWorldBorder();
        if (!worldBorder.isInside(point)) {
            return false;
        }
        // Can't path in an unloaded chunk
        final Chunk chunk = instance.getChunkAt(point);
        if (!ChunkUtils.isLoaded(chunk)) {
            return false;
        }

        this.minimumDistance = minimumDistance;
        if (this.entity.getPosition().distance(point) < minimumDistance) {
            if (onComplete != null) onComplete.accept(null);
            return false;
        }

        if (goalPosition != null && point.sameBlock(goalPosition)) {
            if (onComplete != null) onComplete.accept(null);
            return false;
        }

        this.path = PathGenerator.generate(instance,
                        this.entity.getPosition(),
                        point,
                        minimumDistance, maxDistance,
                        pathVariance,
                this.entity.getBoundingBox(),
                new PPath.PathfinderCapabilities(type, true, true, 0.4f), onComplete);

        final boolean success = path != null;
        this.goalPosition = success ? point : null;
        return success;
    }

    @ApiStatus.Internal
    public synchronized void tick() {
        if (goalPosition == null) return; // No path
        if (entity instanceof LivingEntity && ((LivingEntity) entity).isDead()) return; // No pathfinding tick for dead entities
        if (path == null) return;

        if (path.getState() == PPath.PathState.COMPUTED) {
            path.setState(PPath.PathState.FOLLOWING);
            // Remove nodes that are too close to the start. Prevents doubling back to hit points that have already been hit
            for (int i = 0; i < path.getNodes().size(); i++) {
                if (path.getNodes().get(i).point.sameBlock(entity.getPosition())) {
                    path.getNodes().subList(0, i).clear();
                    break;
                }
            }
        }
        if (path.getState() != PPath.PathState.FOLLOWING) return;

        if (this.entity.getPosition().distance(goalPosition) < minimumDistance) {
            path.runComplete();
            path = null;

            return;
        }

        Point currentTarget = path.getCurrent();

        if (currentTarget == null || path.getCurrentType() == PNode.NodeType.REPATH || path.getCurrentType() == null) {
            path = PathGenerator.generate(entity.getInstance(),
                    entity.getPosition(),
                    Pos.fromPoint(goalPosition),
                    minimumDistance, path.maxDistance(),
                    path.pathVariance(), entity.getBoundingBox(), path.capabilities(), null);

            return;
        }

        moveTowards(currentTarget, movementSpeed, path.capabilities());

        if ((path.getCurrentType() == PNode.NodeType.JUMP || currentTarget.y() > entity.getPosition().y() + 0.1)
                && entity.isOnGround()
                && path.capabilities().canJump()
        ) {
            jump(4f);
        }

        drawPath(path);

        if (entity.getPosition().sameBlock(currentTarget)) path.next();
    }

    /**
     * Gets the target pathfinder position.
     *
     * @return the target pathfinder position, null if there is no one
     */
    public @Nullable Point getGoalPosition() {
        return goalPosition;
    }

    public @NotNull Entity getEntity() {
        return entity;
    }

    public void reset() {
        if (this.path != null) this.path.setState(PPath.PathState.TERMINATING);
        this.goalPosition = null;
        this.path = null;
    }

    public boolean isComplete() {
        if (this.path == null) return true;
        return goalPosition == null || entity.getPosition().sameBlock(goalPosition);
    }

    public List<PNode> getNodes() {
        if (this.path == null) return null;
        return this.path.getNodes();
    }

    public Point getPathPosition() {
        return goalPosition;
    }

    /**
     * Visualise path for debugging
     * @param path the path to draw
     */
    private void drawPath(PPath path) {
        if (path == null) return;

        for (PNode point : path.getNodes()) {
            Point pos = point.point();
            var packet = ParticleCreator.createParticlePacket(Particle.COMPOSTER, pos.x(), pos.y() + 0.5, pos.z(), 0, 0, 0, 1);
            entity.sendPacketToViewers(packet);
        }
    }
}
