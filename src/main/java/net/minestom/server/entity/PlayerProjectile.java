package net.minestom.server.entity;

import net.minestom.server.MinecraftServer;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.collision.PhysicsResult;
import net.minestom.server.collision.ShapeImpl;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.metadata.ProjectileMeta;
import net.minestom.server.event.EventDispatcher;
import net.minestom.server.event.entity.EntityShootEvent;
import net.minestom.server.event.entity.projectile.ProjectileCollideWithBlockEvent;
import net.minestom.server.event.entity.projectile.ProjectileCollideWithEntityEvent;
import net.minestom.server.instance.block.Block;
import org.jetbrains.annotations.NotNull;

import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

public class PlayerProjectile extends LivingEntity {
    private final Entity shooter;
    private long cooldown = 0;

    public PlayerProjectile(Entity shooter, EntityType type) {
        super(type);
        this.shooter = shooter;
        setup();
    }

    private void setup() {
        super.hasPhysics = false;
        if (getEntityMeta() instanceof ProjectileMeta) {
            ((ProjectileMeta) getEntityMeta()).setShooter(this.shooter);
        }
    }

    public void shoot(Point from, double power, double spread) {
        var to = from.add(shooter.getPosition().direction());
        shoot(from, to, power, spread);
    }

    public void shoot(@NotNull Point from, @NotNull Point to, double power, double spread) {
        var instance = shooter.getInstance();
        if (instance == null) return;

        float yaw = -shooter.getPosition().yaw();
        float pitch = -shooter.getPosition().pitch();

        double pitchDiff = pitch - 45;
        if (pitchDiff == 0) pitchDiff = 0.0001;
        double pitchAdjust = pitchDiff * 0.002145329238474369D;

        double dx = to.x() - from.x();
        double dy = to.y() - from.y() + pitchAdjust;
        double dz = to.z() - from.z();
        if (!hasNoGravity()) {
            final double xzLength = Math.sqrt(dx * dx + dz * dz);
            dy += xzLength * 0.20000000298023224D;
        }

        final double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        dx /= length;
        dy /= length;
        dz /= length;
        Random random = ThreadLocalRandom.current();
        spread *= 0.007499999832361937D;
        dx += random.nextGaussian() * spread;
        dy += random.nextGaussian() * spread;
        dz += random.nextGaussian() * spread;

        final EntityShootEvent shootEvent = new EntityShootEvent(this.shooter, this, from, power, spread);
        EventDispatcher.call(shootEvent);
        if (shootEvent.isCancelled()) {
            remove();
            return;
        }

        final double mul = 20 * power;
        Vec v = new Vec(dx * mul, dy * mul * 0.9, dz * mul);

        this.setInstance(instance, new Pos(from.x(), from.y(), from.z(), yaw, pitch)).whenComplete((result, throwable) -> {
            if (throwable != null) {
                throwable.printStackTrace();
            } else {
                this.setVelocity(v);
            }
        });

        cooldown = System.currentTimeMillis();
    }

    @Override
    public void refreshPosition(@NotNull Pos newPosition) {
    }

    @Override
    public void tick(long time) {
        final Pos posBefore = getPosition();
        super.tick(time);
        final Pos posNow = getPosition();

        Vec diff = Vec.fromPoint(posNow.sub(posBefore));

        if (cooldown + 500 < System.currentTimeMillis()) {
            float yaw = (float) Math.toDegrees(Math.atan2(diff.x(), diff.z()));
            float pitch = (float) Math.toDegrees(Math.atan2(diff.y(), Math.sqrt(diff.x() * diff.x() + diff.z() * diff.z())));
            super.refreshPosition(new Pos(posNow.x(), posNow.y(), posNow.z(), yaw, pitch));
            cooldown = System.currentTimeMillis();
        }

        Entity collided = CollisionUtils.checkEntityCollisions(this, diff, 3);
        if (collided != null && collided != shooter && collided != this) {
            var e = new ProjectileCollideWithEntityEvent(this, collided.getPosition(), collided);
            MinecraftServer.getGlobalEventHandler().call(e);
            return;
        }

        PhysicsResult result = CollisionUtils.handlePhysics(this, diff, null, false);

        if (result.hasCollision()) {
            Block hitBlock = null;
            Point hitPoint = null;
            if (result.collisionShapes()[0] instanceof ShapeImpl block) {
                hitBlock = block.block();
                hitPoint = result.collisionPoints()[0];
            }
            if (result.collisionShapes()[1] instanceof ShapeImpl block) {
                hitBlock = block.block();
                hitPoint = result.collisionPoints()[1];
            }
            if (result.collisionShapes()[2] instanceof ShapeImpl block) {
                hitBlock = block.block();
                hitPoint = result.collisionPoints()[2];
            }

            if (hitBlock == null) return;

            var e = new ProjectileCollideWithBlockEvent(this, Pos.fromPoint(hitPoint), hitBlock);
            MinecraftServer.getGlobalEventHandler().call(e);
        }
    }
}
