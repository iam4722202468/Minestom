package net.minestom.server.collision;

import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityType;
import net.minestom.server.entity.GameMode;
import net.minestom.server.entity.Player;
import net.minestom.server.entity.metadata.other.ArmorStandMeta;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.server.utils.block.BlockIterator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.Iterator;

final class BlockCollision {
    /**
     * Moves an entity with physics applied (ie checking against blocks)
     * <p>
     * Works by getting all the full blocks that an entity could interact with.
     * All bounding boxes inside the full blocks are checked for collisions with the entity.
     */
    static PhysicsResult handlePhysics(@NotNull BoundingBox boundingBox,
                                       @NotNull Vec velocity, @NotNull Pos entityPosition,
                                       @NotNull Block.Getter getter,
                                       @Nullable PhysicsResult lastPhysicsResult,
                                       boolean singleCollision) {
        if (velocity.isZero()) {
            // TODO should return a constant
            return new PhysicsResult(entityPosition, Vec.ZERO, false, false, false, false, velocity, new Point[3], new Shape[3], false, SweepResult.NO_COLLISION);
        }
        // Fast-exit using cache
        final PhysicsResult cachedResult = cachedPhysics(velocity, entityPosition, getter, lastPhysicsResult);
        if (cachedResult != null) {
            return cachedResult;
        }
        // Expensive AABB computation
        return stepPhysics(boundingBox, velocity, entityPosition, getter, singleCollision);
    }


    static private void checkBlocks(Iterator<Iterator<Point>> iterator, @NotNull Vec velocity, Pos entityPosition, @NotNull BoundingBox boundingBox, @NotNull Block.Getter getter, @NotNull SweepResult finalResult, BoundingBox.AxisMask axis) {
        while (iterator.hasNext()) {
            var i = iterator.next();
            while (i.hasNext()) {
                var checkBlock = i.next();

                // Fast exit if we have already hit axis. Axis order is x, y, z
                if (finalResult.collidedPosition != null && (axis == BoundingBox.AxisMask.Y || axis == BoundingBox.AxisMask.Z)) {
                    if (velocity.x() < 0 && checkBlock.blockX() + 1 < finalResult.collidedPosition.blockX()) return;
                    else if (checkBlock.blockX() > finalResult.collidedPosition.blockX()) return;
                }

                // Needs to be 2 for stairs
                if (finalResult.collidedPosition != null && axis == BoundingBox.AxisMask.Z) {
                    if (velocity.y() < 0 && checkBlock.blockY() + 2 < finalResult.collidedPosition.blockY()) return;
                    else if (checkBlock.blockY() - 2 > finalResult.collidedPosition.blockY()) return;
                }

                checkBoundingBox(checkBlock.blockX(), checkBlock.blockY(), checkBlock.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            }
        }
    }

    private static PhysicsResult computePhysics(@NotNull BoundingBox boundingBox,
                                                @NotNull Vec velocity, Pos entityPosition,
                                                @NotNull Block.Getter getter,
                                                @NotNull SweepResult finalResult) {
        BoundingBoxFace[] face = getFaces(entityPosition, boundingBox, velocity);

        if (face[0] != null) checkBlocks(face[0], velocity, entityPosition, boundingBox, getter, finalResult, BoundingBox.AxisMask.X);
        if (face[1] != null) checkBlocks(face[1], velocity, entityPosition, boundingBox, getter, finalResult, BoundingBox.AxisMask.Y);
        if (face[2] != null) checkBlocks(face[2], velocity, entityPosition, boundingBox, getter, finalResult, BoundingBox.AxisMask.Z);

        final boolean collisionX = finalResult.normalX != 0;
        final boolean collisionY = finalResult.normalY != 0;
        final boolean collisionZ = finalResult.normalZ != 0;

        double deltaX = finalResult.res * velocity.x();
        double deltaY = finalResult.res * velocity.y();
        double deltaZ = finalResult.res * velocity.z();

        if (Math.abs(deltaX) < Vec.EPSILON) deltaX = 0;
        if (Math.abs(deltaY) < Vec.EPSILON) deltaY = 0;
        if (Math.abs(deltaZ) < Vec.EPSILON) deltaZ = 0;

        final Pos finalPos = entityPosition.add(deltaX, deltaY, deltaZ);

        final double remainingX = collisionX ? 0 : velocity.x() - deltaX;
        final double remainingY = collisionY ? 0 : velocity.y() - deltaY;
        final double remainingZ = collisionZ ? 0 : velocity.z() - deltaZ;

        return new PhysicsResult(finalPos, new Vec(remainingX, remainingY, remainingZ),
                collisionY, collisionX, collisionY, collisionZ,
                Vec.ZERO, null, null, false, finalResult);
    }

    static class AxisBlockIterator implements Iterator<Point> {
        private final BoundingBox.AxisMask axis;
        private final int maxX, maxY, minX, minY;
        private final int axisValue;
        private int currentX, currentY;

        public AxisBlockIterator(BoundingBox.AxisMask axis, int minX, int minY, int maxX, int maxY, int axisValue) {
            this.axis = axis;
            this.maxX = Math.max(maxX, minX);
            this.maxY = Math.max(maxY, minY);
            this.minX = Math.min(minX, maxX);
            this.minY = Math.min(minY, maxY);
            currentX = this.minX;
            currentY = this.minY;
            this.axisValue = axisValue;
        }

        @Override
        public boolean hasNext() {
            return currentX <= maxX && currentY <= maxY;
        }

        @Override
        public Point next() {
            Point p = switch(axis) {
                case X -> new Vec(axisValue, currentX, currentY);
                case Y -> new Vec(currentX, axisValue, currentY);
                case Z -> new Vec(currentX, currentY, axisValue);
                case NONE -> throw new IllegalStateException("NONE axis is not supported");
            };

            if (currentX == maxX) {
                currentX = minX;
                currentY++;
            } else {
                currentX++;
            }

            return p;
        }
    }

    static private int min(int a, int b, int c, int d) {
        return Math.min(Math.min(a, b), Math.min(c, d));
    }

    static private int max(int a, int b, int c, int d) {
        return Math.max(Math.max(a, b), Math.max(c, d));
    }

    static class BoundingBoxFace implements Iterator<Iterator<Point>> {
        private final Vec direction;
        private final Pos[] corners;
        private final BoundingBox.AxisMask axis;
        private final BlockIterator i1, i2;

        private Point i1p, i2p;
        private int currentBlock;
        private int endBlock;

        public BoundingBoxFace(Pos[] corners, Pos start, Vec velocity, BoundingBox.AxisMask axis) {
            this.corners = corners;

            this.axis = axis;
            this.direction = new Vec(Math.signum(velocity.x()), Math.signum(velocity.y()), Math.signum(velocity.z()));

            if (axis == BoundingBox.AxisMask.X) {
                currentBlock = (int) Math.floor(corners[0].blockX());
                endBlock = (int) Math.floor(currentBlock + velocity.x()) + getAxisBlock(direction) * 2;
            } else if (axis == BoundingBox.AxisMask.Y) {
                currentBlock = (int) Math.floor(corners[0].blockY());
                endBlock = (int) Math.floor(currentBlock + velocity.y()) + getAxisBlock(direction) * 2;
            } else if (axis == BoundingBox.AxisMask.Z) {
                currentBlock = (int) Math.floor(corners[0].blockZ());
                endBlock = (int) Math.floor(currentBlock + velocity.z()) + getAxisBlock(direction) * 2;
            }

            i1 = new BlockIterator(corners[0].asVec(), velocity, 0, velocity.length(), false);
            i2 = new BlockIterator(corners[1].asVec(), velocity, 0, velocity.length(), false);

            i1p = corners[0].asVec();
            i2p = corners[1].asVec();
        }

        @Override
        public String toString() {
            return "BoundingBoxFace{" +
                    "corners=" + Arrays.toString(corners) +
                    '}';
        }

        private int getAxisBlock(Point p) {
            if (axis == BoundingBox.AxisMask.X) return (int) Math.floor(p.x());
            else if (axis == BoundingBox.AxisMask.Y) return (int) Math.floor(p.y());
            else if (axis == BoundingBox.AxisMask.Z) return (int) Math.floor(p.z());
            else throw new IllegalStateException("Invalid axis mask");
        }

        public Iterator<Point> next() {
            int target = currentBlock + getAxisBlock(direction);

            Point li1p = i1p;
            Point li2p = i2p;

            Point initiali1p = i1p;
            Point initiali2p = i2p;

            while (getAxisBlock(i1p) != target && i1.hasNext()) {
                li1p = i1p;
                i1p = i1.next();
            }

            while (getAxisBlock(i2p) != target && i2.hasNext()) {
                li2p = i2p;
                i2p = i2.next();
            }

            // 2d
            int maxX, maxY, minX, minY;

            if (axis == BoundingBox.AxisMask.X) {
                minX = min(initiali1p.blockY(), initiali2p.blockY(), li1p.blockY(), li2p.blockY());
                minY = min(initiali1p.blockZ(), initiali2p.blockZ(), li1p.blockZ(), li2p.blockZ());
                maxX = max(li1p.blockY(), li2p.blockY(), initiali1p.blockY(), initiali2p.blockY());
                maxY = max(li1p.blockZ(), li2p.blockZ(), initiali1p.blockZ(), initiali2p.blockZ());
            } else if (axis == BoundingBox.AxisMask.Y) {
                minX = min(initiali1p.blockX(), initiali2p.blockX(), li1p.blockX(), li2p.blockX());
                minY = min(initiali1p.blockZ(), initiali2p.blockZ(), li1p.blockZ(), li2p.blockZ());
                maxX = max(li1p.blockX(), li2p.blockX(), initiali1p.blockX(), initiali2p.blockX());
                maxY = max(li1p.blockZ(), li2p.blockZ(), initiali1p.blockZ(), initiali2p.blockZ());
            } else if (axis == BoundingBox.AxisMask.Z) {
                minX = min(initiali1p.blockX(), initiali2p.blockX(), li1p.blockX(), li2p.blockX());
                minY = min(initiali1p.blockY(), initiali2p.blockY(), li1p.blockY(), li2p.blockY());
                maxX = max(li1p.blockX(), li2p.blockX(), initiali1p.blockX(), initiali2p.blockX());
                maxY = max(li1p.blockY(), li2p.blockY(), initiali1p.blockY(), initiali2p.blockY());
            } else throw new IllegalStateException("Invalid axis mask");

            var res = new AxisBlockIterator(axis, minX, minY, maxX, maxY, currentBlock);
            currentBlock = target;
            return res;
        }

        @Override
        public boolean hasNext() {
            return currentBlock != endBlock;
        }
    }

    private static BoundingBoxFace[] getFaces(Pos position, BoundingBox boundingBox, Vec velocity) {
        Pos[] corners = new Pos[8];

        corners[0] = position.add(new Vec(boundingBox.minX(), boundingBox.minY(), boundingBox.minZ()));
        corners[1] = position.add(new Vec(boundingBox.minX(), boundingBox.minY(), boundingBox.maxZ()));
        corners[2] = position.add(new Vec(boundingBox.minX(), boundingBox.maxY(), boundingBox.minZ()));
        corners[3] = position.add(new Vec(boundingBox.minX(), boundingBox.maxY(), boundingBox.maxZ()));
        corners[4] = position.add(new Vec(boundingBox.maxX(), boundingBox.minY(), boundingBox.minZ()));
        corners[5] = position.add(new Vec(boundingBox.maxX(), boundingBox.minY(), boundingBox.maxZ()));
        corners[6] = position.add(new Vec(boundingBox.maxX(), boundingBox.maxY(), boundingBox.minZ()));
        corners[7] = position.add(new Vec(boundingBox.maxX(), boundingBox.maxY(), boundingBox.maxZ()));

        BoundingBoxFace[] faces = new BoundingBoxFace[3];

        if (Math.abs(velocity.x()) > Vec.EPSILON) {
            if (velocity.x() > 0) {
                faces[0] = new BoundingBoxFace(new Pos[]{corners[4], corners[7]}, position, velocity, BoundingBox.AxisMask.X);
            } else if (velocity.x() < 0) {
                faces[0] = new BoundingBoxFace(new Pos[]{corners[0], corners[3]}, position, velocity, BoundingBox.AxisMask.X);
            }
        }

        if (Math.abs(velocity.y()) > Vec.EPSILON) {
            if (velocity.y() > 0) {
                faces[1] = new BoundingBoxFace(new Pos[]{corners[2], corners[7]}, position, velocity, BoundingBox.AxisMask.Y);
            } else if (velocity.y() < 0) {
                faces[1] = new BoundingBoxFace(new Pos[]{corners[0], corners[5]}, position, velocity, BoundingBox.AxisMask.Y);
            }
        }

        if (Math.abs(velocity.z()) > Vec.EPSILON) {
            if (velocity.z() > 0)
                faces[2] = new BoundingBoxFace(new Pos[]{corners[1], corners[7]}, position, velocity, BoundingBox.AxisMask.Z);
            else if (velocity.z() < 0)
                faces[2] = new BoundingBoxFace(new Pos[]{corners[0], corners[6]}, position, velocity, BoundingBox.AxisMask.Z);
        }

        return faces;
    }

    static Entity canPlaceBlockAt(Instance instance, Point blockPos, Block b) {
        for (Entity entity : instance.getNearbyEntities(blockPos, 3)) {
            final EntityType type = entity.getEntityType();
            if (type == EntityType.ITEM || type == EntityType.ARROW)
                continue;
            // Marker Armor Stands should not prevent block placement
            if (entity.getEntityMeta() instanceof ArmorStandMeta armorStandMeta && armorStandMeta.isMarker())
                continue;

            final boolean intersects;
            if (entity instanceof Player) {
                // Ignore spectators
                if (((Player) entity).getGameMode() == GameMode.SPECTATOR)
                    continue;
                // Need to move player slightly away from block we're placing.
                // If player is at block 40 we cannot place a block at block 39 with side length 1 because the block will be in [39, 40]
                // For this reason we subtract a small amount from the player position
                Point playerPos = entity.getPosition().add(entity.getPosition().sub(blockPos).mul(0.0000001));
                intersects = b.registry().collisionShape().intersectBox(playerPos.sub(blockPos), entity.getBoundingBox());
            } else {
                intersects = b.registry().collisionShape().intersectBox(entity.getPosition().sub(blockPos), entity.getBoundingBox());
            }
            if (intersects) return entity;
        }
        return null;
    }

    private static PhysicsResult cachedPhysics(Vec velocity, Pos entityPosition,
                                               Block.Getter getter, PhysicsResult lastPhysicsResult) {
        if (lastPhysicsResult != null && lastPhysicsResult.collisionShapes()[1] instanceof ShapeImpl shape) {
            Block collisionBlockY = shape.block();

            // Fast exit if entity hasn't moved
            if (lastPhysicsResult.collisionY()
                    && velocity.y() == lastPhysicsResult.originalDelta().y()
                    // Check block below to fast exit gravity
                    && getter.getBlock(lastPhysicsResult.collisionPoints()[1].sub(0, Vec.EPSILON, 0), Block.Getter.Condition.TYPE) == collisionBlockY
                    && velocity.x() == 0 && velocity.z() == 0
                    && entityPosition.samePoint(lastPhysicsResult.newPosition())
                    && collisionBlockY != Block.AIR) {
                return lastPhysicsResult;
            }
        }
        return null;
    }

    private static PhysicsResult stepPhysics(@NotNull BoundingBox boundingBox,
                                             @NotNull Vec velocity, @NotNull Pos entityPosition,
                                             @NotNull Block.Getter getter, boolean singleCollision) {
        // Allocate once and update values
        SweepResult finalResult = new SweepResult(1 - Vec.EPSILON, 0, 0, 0, null, null);

        boolean foundCollisionX = false, foundCollisionY = false, foundCollisionZ = false;

        Point[] collidedPoints = new Point[3];
        Shape[] collisionShapes = new Shape[3];

        boolean hasCollided = false;

        // Query faces to get the points needed for collision
        PhysicsResult result = computePhysics(boundingBox, velocity, entityPosition, getter, finalResult);
        // Loop until no collisions are found.
        // When collisions are found, the collision axis is set to 0
        // Looping until there are no collisions will allow the entity to move in axis other than the collision axis after a collision.
        while (result.collisionX() || result.collisionY() || result.collisionZ()) {
            // Reset final result
            finalResult.normalX = 0;
            finalResult.normalY = 0;
            finalResult.normalZ = 0;

            if (result.collisionX()) {
                foundCollisionX = true;
                collisionShapes[0] = finalResult.collidedShape;
                collidedPoints[0] = finalResult.collidedPosition;
                hasCollided = true;
                if (singleCollision) break;
            } else if (result.collisionZ()) {
                foundCollisionZ = true;
                collisionShapes[2] = finalResult.collidedShape;
                collidedPoints[2] = finalResult.collidedPosition;
                hasCollided = true;
                if (singleCollision) break;
            } else if (result.collisionY()) {
                foundCollisionY = true;
                collisionShapes[1] = finalResult.collidedShape;
                collidedPoints[1] = finalResult.collidedPosition;
                hasCollided = true;
                if (singleCollision) break;
            }

            // If all axis have had collisions, break
            if (foundCollisionX && foundCollisionY && foundCollisionZ) break;
            // If the entity isn't moving, break
            if (result.newVelocity().isZero()) break;

            finalResult.res = 1 - Vec.EPSILON;
            result = computePhysics(boundingBox, result.newVelocity(), result.newPosition(), getter, finalResult);
        }

        finalResult.res = result.res().res;

        final double newDeltaX = foundCollisionX ? 0 : velocity.x();
        final double newDeltaY = foundCollisionY ? 0 : velocity.y();
        final double newDeltaZ = foundCollisionZ ? 0 : velocity.z();

        return new PhysicsResult(result.newPosition(), new Vec(newDeltaX, newDeltaY, newDeltaZ),
                newDeltaY == 0 && velocity.y() < 0,
                foundCollisionX, foundCollisionY, foundCollisionZ, velocity, collidedPoints, collisionShapes, hasCollided, finalResult);
    }

    private static void slowPhysics(@NotNull BoundingBox boundingBox,
                                    @NotNull Vec velocity, Pos entityPosition,
                                    @NotNull Block.Getter getter,
                                    @NotNull Vec[] allFaces,
                                    @NotNull SweepResult finalResult) {
        BoundingBoxFace[] face = getFaces(entityPosition, boundingBox, velocity);

        if (face[0] != null) checkBlocks(face[0], velocity, entityPosition, boundingBox, getter, finalResult, BoundingBox.AxisMask.X);
        if (face[1] != null) checkBlocks(face[1], velocity, entityPosition, boundingBox, getter, finalResult, BoundingBox.AxisMask.Y);
        if (face[2] != null) checkBlocks(face[2], velocity, entityPosition, boundingBox, getter, finalResult, BoundingBox.AxisMask.Z);
    }

    private static void fastPhysics(@NotNull BoundingBox boundingBox,
                                    @NotNull Vec velocity, Pos entityPosition,
                                    @NotNull Block.Getter getter,
                                    @NotNull Vec[] allFaces,
                                    @NotNull SweepResult finalResult) {
        for (Vec point : allFaces) {
            final Vec pointBefore = point.add(entityPosition);
            final Vec pointAfter = point.add(entityPosition).add(velocity);
            // Entity can pass through up to 4 blocks. Starting block, Two intermediate blocks, and a final block.
            // This means we must check every combination of block movements when an entity moves over an axis.
            // 000, 001, 010, 011, etc.
            // There are 8 of these combinations
            // Checks can be limited by checking if we moved across an axis line

            boolean needsX = pointBefore.x() != pointAfter.x();
            boolean needsY = pointBefore.y() != pointAfter.y();
            boolean needsZ = pointBefore.z() != pointAfter.z();

            checkBoundingBox(pointBefore.blockX(), pointBefore.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);

            if (needsX && needsY && needsZ) {
                checkBoundingBox(pointAfter.blockX(), pointAfter.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);

                checkBoundingBox(pointAfter.blockX(), pointAfter.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointAfter.blockX(), pointBefore.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointBefore.blockX(), pointAfter.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);

                checkBoundingBox(pointAfter.blockX(), pointBefore.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointBefore.blockX(), pointAfter.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointBefore.blockX(), pointBefore.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            } else if (needsX && needsY) {
                checkBoundingBox(pointAfter.blockX(), pointAfter.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);

                checkBoundingBox(pointAfter.blockX(), pointBefore.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointBefore.blockX(), pointAfter.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            } else if (needsX && needsZ) {
                checkBoundingBox(pointAfter.blockX(), pointBefore.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);

                checkBoundingBox(pointAfter.blockX(), pointBefore.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointBefore.blockX(), pointBefore.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            } else if (needsY && needsZ) {
                checkBoundingBox(pointBefore.blockX(), pointAfter.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);

                checkBoundingBox(pointBefore.blockX(), pointAfter.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
                checkBoundingBox(pointBefore.blockX(), pointBefore.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            } else if (needsX) {
                checkBoundingBox(pointAfter.blockX(), pointBefore.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            } else if (needsY) {
                checkBoundingBox(pointBefore.blockX(), pointAfter.blockY(), pointBefore.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            } else if (needsZ) {
                checkBoundingBox(pointBefore.blockX(), pointBefore.blockY(), pointAfter.blockZ(), velocity, entityPosition, boundingBox, getter, finalResult);
            }
        }
    }

    /**
     * Check if a moving entity will collide with a block. Updates finalResult
     *
     * @param blockX         block x position
     * @param blockY         block y position
     * @param blockZ         block z position
     * @param entityVelocity entity movement vector
     * @param entityPosition entity position
     * @param boundingBox    entity bounding box
     * @param getter         block getter
     * @param finalResult    place to store final result of collision
     * @return true if entity finds collision, other false
     */
    static boolean checkBoundingBox(int blockX, int blockY, int blockZ,
                                    Vec entityVelocity, Pos entityPosition, BoundingBox boundingBox,
                                    Block.Getter getter, SweepResult finalResult) {
        // Don't step if chunk isn't loaded yet
        final Block currentBlock = getter.getBlock(blockX, blockY, blockZ, Block.Getter.Condition.TYPE);
        final Shape currentShape = currentBlock.registry().collisionShape();

        final boolean currentCollidable = !currentShape.relativeEnd().isZero();
        final boolean currentShort = currentShape.relativeEnd().y() < 0.5;

        // only consider the block below if our current shape is sufficiently short
        if (currentShort && shouldCheckLower(entityVelocity, entityPosition, blockX, blockY, blockZ)) {
            // we need to check below for a tall block (fence, wall, ...)
            final Vec belowPos = new Vec(blockX, blockY - 1, blockZ);
            final Block belowBlock = getter.getBlock(belowPos, Block.Getter.Condition.TYPE);
            final Shape belowShape = belowBlock.registry().collisionShape();

            final Vec currentPos = new Vec(blockX, blockY, blockZ);
            // don't fall out of if statement, we could end up redundantly grabbing a block, and we only need to
            // collision check against the current shape since the below shape isn't tall
            if (belowShape.relativeEnd().y() > 1) {
                // we should always check both shapes, so no short-circuit here, to handle cases where the bounding box
                // hits the current solid but misses the tall solid
                return belowShape.intersectBoxSwept(entityPosition, entityVelocity, belowPos, boundingBox, finalResult) |
                        (currentCollidable && currentShape.intersectBoxSwept(entityPosition, entityVelocity, currentPos, boundingBox, finalResult));
            } else {
                return currentCollidable && currentShape.intersectBoxSwept(entityPosition, entityVelocity, currentPos, boundingBox, finalResult);
            }
        }

        if (currentCollidable && currentShape.intersectBoxSwept(entityPosition, entityVelocity,
                new Vec(blockX, blockY, blockZ), boundingBox, finalResult)) {
            // if the current collision is sufficiently short, we might need to collide against the block below too
            if (currentShort) {
                final Vec belowPos = new Vec(blockX, blockY - 1, blockZ);
                final Block belowBlock = getter.getBlock(belowPos, Block.Getter.Condition.TYPE);
                final Shape belowShape = belowBlock.registry().collisionShape();
                // only do sweep if the below block is big enough to possibly hit
                if (belowShape.relativeEnd().y() > 1)
                    belowShape.intersectBoxSwept(entityPosition, entityVelocity, belowPos, boundingBox, finalResult);
            }
            return true;
        }
        return false;
    }

    private static boolean shouldCheckLower(Vec entityVelocity, Pos entityPosition, int blockX, int blockY, int blockZ) {
        final double yVelocity = entityVelocity.y();
        // if moving horizontally, just check if the floor of the entity's position is the same as the blockY
        if (yVelocity == 0) return Math.floor(entityPosition.y()) == blockY;
        final double xVelocity = entityVelocity.x();
        final double zVelocity = entityVelocity.z();
        // if moving straight up, don't bother checking for tall solids beneath anything
        // if moving straight down, only check for a tall solid underneath the last block
        if (xVelocity == 0 && zVelocity == 0)
            return yVelocity < 0 && blockY == Math.floor(entityPosition.y() + yVelocity);
        // default to true: if no x velocity, only consider YZ line, and vice-versa
        final boolean underYX = xVelocity != 0 && computeHeight(yVelocity, xVelocity, entityPosition.y(), entityPosition.x(), blockX) >= blockY;
        final boolean underYZ = zVelocity != 0 && computeHeight(yVelocity, zVelocity, entityPosition.y(), entityPosition.z(), blockZ) >= blockY;
        // true if the block is at or below the same height as a line drawn from the entity's position to its final
        // destination
        return underYX && underYZ;
    }

    /*
    computes the height of the entity at the given block position along a projection of the line it's travelling along
    (YX or YZ). the returned value will be greater than or equal to the block height if the block is along the lower
    layer of intersections with this line.
     */
    private static double computeHeight(double yVelocity, double velocity, double entityY, double pos, int blockPos) {
        final double m = yVelocity / velocity;
        /*
        offsetting by 1 is necessary with a positive slope, because we can clip the bottom-right corner of blocks
        without clipping the "bottom-left" (the smallest corner of the block on the YZ or YX plane). without the offset
        these would not be considered to be on the lowest layer, since our block position represents the bottom-left
        corner
         */
        return m * (blockPos - pos + (m > 0 ? 1 : 0)) + entityY;
    }
}