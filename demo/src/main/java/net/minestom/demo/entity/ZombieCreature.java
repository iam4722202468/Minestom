package net.minestom.demo.entity;

import net.minestom.server.attribute.Attribute;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.*;
import net.minestom.server.entity.ai.EntityAIGroupBuilder;
import net.minestom.server.entity.ai.goal.FollowTargetGoal;
import net.minestom.server.entity.ai.target.ClosestEntityTarget;
import net.minestom.server.entity.metadata.other.InteractionMeta;
import net.minestom.server.instance.Instance;
import org.jetbrains.annotations.NotNull;

import java.util.concurrent.CompletableFuture;

public class ZombieCreature extends EntityCreature {
    private final LivingEntity e;

    public ZombieCreature() {
        super(EntityType.ZOMBIE);
        this.getAttribute(Attribute.MOVEMENT_SPEED).setBaseValue(0.15f);
        // this.setNoGravity(true);
        this.setBoundingBox(1, 1, 1);

        this.e = new LivingEntity(EntityType.INTERACTION);
        InteractionMeta meta = (InteractionMeta) e.getEntityMeta();
        meta.setHeight(1f);
        meta.setWidth(1f);

        addAIGroup(
                new EntityAIGroupBuilder()
                        // .addTargetSelector(new ClosestEntityTarget(this, 500, Player.class))
                        // .addGoalSelector(new FollowTargetGoal(this, Duration.ofMillis(500)))
                        .build()
        );
    }

    @Override
    public CompletableFuture<Void> setInstance(@NotNull Instance instance, @NotNull Pos spawnPosition) {
        e.setInstance(instance, spawnPosition);
        return super.setInstance(instance, spawnPosition);
    }

    @Override
    public void tick(long time) {
        super.tick(time);
        e.setNoGravity(true);
        e.teleport(getPosition());
    }
}