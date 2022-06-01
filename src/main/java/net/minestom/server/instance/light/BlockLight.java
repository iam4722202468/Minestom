package net.minestom.server.instance.light;

import it.unimi.dsi.fastutil.ints.IntArrayFIFOQueue;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.Chunk;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.Section;
import net.minestom.server.instance.block.Block;
import net.minestom.server.instance.block.BlockFace;
import net.minestom.server.instance.palette.Palette;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import static net.minestom.server.instance.light.BlockLightCompute.*;

final class BlockLight implements Light {
    private final Palette blockPalette;

    private byte[] content;
    private byte[] contentPropagation;
    private byte[] contentPropagationSwap;
    private byte[] baked;

    private byte[][] borders;
    private byte[][] bordersPropagation;
    private byte[][] bordersPropagationSwap;
    private boolean isValid = false;
    private Set<Point> toUpdateSet = new HashSet<>();
    private boolean isValidBase = true;

    BlockLight(Palette blockPalette) {
        this.blockPalette = blockPalette;
    }

    @Override
    public Set<Point> flip() {
        if (this.bordersPropagationSwap != null)
            this.bordersPropagation = this.bordersPropagationSwap;

        if (this.contentPropagationSwap != null)
            this.contentPropagation = this.contentPropagationSwap;

        this.bordersPropagationSwap = null;
        this.contentPropagationSwap = null;

        if (toUpdateSet == null) return Set.of();
        return toUpdateSet;
    }

    static IntArrayFIFOQueue buildInternalQueue(Palette blockPalette, Block[] blocks) {
        IntArrayFIFOQueue lightSources = new IntArrayFIFOQueue();
        // Apply section light
        blockPalette.getAllPresent((x, y, z, stateId) -> {
            final Block block = Block.fromStateId((short) stateId);
            assert block != null;
            final byte lightEmission = (byte) block.registry().lightEmission();

            final int index = x | (z << 4) | (y << 8);
            blocks[index] = block;
            if (lightEmission > 0) {
                lightSources.enqueue(index | (lightEmission << 12));
            }
        });
        return lightSources;
    }

    private static Block getBlock(Palette palette, int x, int y, int z) {
        return Block.fromStateId((short)palette.get(x, y, z));
    }

    private static IntArrayFIFOQueue buildExternalQueue(Instance instance, Block[] blocks, Map<BlockFace, Point> neighbors) {
        IntArrayFIFOQueue lightSources = new IntArrayFIFOQueue();

        for (BlockFace face : BlockFace.values()) {
            Point neighborSection = neighbors.get(face);
            if (neighborSection == null) continue;

            Chunk chunk = instance.getChunk(neighborSection.blockX(), neighborSection.blockZ());
            if (chunk == null) continue;

            byte[] neighborFace = chunk.getSection(neighborSection.blockY()).blockLight().getBorderPropagation(face.getOppositeFace());
            if (neighborFace == null) continue;

            for (int bx = 0; bx < 16; bx++) {
                for (int by = 0; by < 16; by++) {
                    final int k = switch (face) {
                        case WEST, BOTTOM, NORTH -> 0;
                        case EAST, TOP, SOUTH -> 15;
                    };

                    final int posTo = switch (face) {
                        case NORTH, SOUTH -> bx | (k << 4) | (by << 8);
                        case WEST, EAST -> k | (by << 4) | (bx << 8);
                        default -> bx | (by << 4) | (k << 8);
                    };

                    Section otherSection = chunk.getSection(neighborSection.blockY());

                    final Block blockFrom = (switch (face) {
                        case NORTH, SOUTH -> getBlock(otherSection.blockPalette(), bx, by, 15 - k);
                        case WEST, EAST -> getBlock(otherSection.blockPalette(), 15 - k, bx, by);
                        default -> getBlock(otherSection.blockPalette(), bx, 15 - k, by);
                    });

                    if (blocks == null) continue;
                    Block blockTo = blocks[posTo];

                    if (blockTo == null && blockFrom != null) {
                        if (blockFrom.registry().collisionShape().isOccluded(Block.AIR.registry().collisionShape(), face.getOppositeFace()))
                            continue;
                    } else if (blockTo != null && blockFrom == null) {
                        if (Block.AIR.registry().collisionShape().isOccluded(blockTo.registry().collisionShape(), face))
                            continue;
                    } else if (blockTo != null && blockFrom != null) {
                        if (blockFrom.registry().collisionShape().isOccluded(blockTo.registry().collisionShape(), face.getOppositeFace()))
                            continue;
                    }

                    final int borderIndex = bx * SECTION_SIZE + by;
                    byte lightEmission = neighborFace[borderIndex];

                    final int index = posTo | (lightEmission << 12);

                    if (lightEmission > 0) {
                        lightSources.enqueue(index);
                    }
                }
            }
        }

        return lightSources;
    }

    @Override
    public void copyFrom(byte @NotNull [] array) {
        if (array.length == 0) this.content = null;
        else this.content = array.clone();
    }

    @Override
    public Light calculateInternal(Instance instance, int chunkX, int sectionY, int chunkZ) {
        Chunk chunk = instance.getChunk(chunkX, chunkZ);
        if (chunk == null) {
            this.toUpdateSet = Set.of();
            return this;
        }

        if (this.isValidBase) {
            this.toUpdateSet = Set.of();
            return this;
        }
        this.isValidBase = true;

        Set<Point> toUpdate = new HashSet<>();

        // Update single section with base lighting changes
        Block[] blocks = new Block[SECTION_SIZE * SECTION_SIZE * SECTION_SIZE];
        IntArrayFIFOQueue queue = buildInternalQueue(blockPalette, blocks);

        Result result = BlockLightCompute.compute(blocks, queue);
        this.content = result.light();
        this.borders = new byte[FACES.length][];
        Arrays.setAll(borders, i -> new byte[SIDE_LENGTH]);

        // Propagate changes to neighbors and self
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                Chunk neighborChunk = instance.getChunk(chunkX + i, chunkZ + j);
                if (neighborChunk == null) continue;

                for (int k = -1; k <= 1; k++) {
                    Vec neighborPos = new Vec(chunkX + i, sectionY + k, chunkZ + j);

                    if (neighborPos.blockY() >= neighborChunk.getMinSection() && neighborPos.blockY() < neighborChunk.getMaxSection()) {
                        toUpdate.add(new Vec(neighborChunk.getChunkX(), neighborPos.blockY(), neighborChunk.getChunkZ()));
                        neighborChunk.getSection(neighborPos.blockY()).blockLight().invalidatePropagation();
                    }
                }
            }
        }

        toUpdate.add(new Vec(chunk.getChunkX(), sectionY, chunk.getChunkZ()));
        this.borders = result.borders();
        this.toUpdateSet = toUpdate;

        return this;
    }

    @Override
    public void invalidate() {
        this.isValidBase = false;
        invalidatePropagation();
    }

    private void clearCache() {
        this.contentPropagation = null;
        this.bordersPropagation = null;
        baked = null;
        isValid = true;
    }

    @Override
    public byte[] array() {
        if (baked != null) return baked;
        if (!isValid) clearCache();
        freePropagation();
        return baked;
    }

    private boolean compareBorders(byte[] a, byte[] b) {
        if (b == null && a == null) return true;
        if (b == null || a == null) return false;

        if (a.length != b.length) return false;
        for (int i = 0; i < a.length; i++) {
            if (a[i] > b[i]) return false;
        }
        return true;
    }

    public Block[] blocks() {
        Block[] blocks = new Block[SECTION_SIZE * SECTION_SIZE * SECTION_SIZE];

        blockPalette.getAllPresent((x, y, z, stateId) -> {
            final Block block = Block.fromStateId((short) stateId);
            assert block != null;
            final int index = x | (z << 4) | (y << 8);
            blocks[index] = block;
        });

        return blocks;
    }

    @Override
    public Light calculateExternal(Instance instance, Chunk chunk, int sectionY) {
        if (!isValid) clearCache();

        var neighbors = instance.getNeighbors(chunk, sectionY);
        Set<Point> toUpdate = new HashSet<>();

        Block[] blocks = blocks();

        IntArrayFIFOQueue queue = buildExternalQueue(instance, blocks, neighbors);

        BlockLightCompute.Result result = BlockLightCompute.compute(blocks, queue);

        byte[] contentPropagationTemp = result.light();
        byte[][] borderTemp = result.borders();

        this.contentPropagationSwap = bake(contentPropagation, contentPropagationTemp);

        // Propagate changes to neighbors and self
        for (var entry : neighbors.entrySet()) {
            var neighbor = entry.getValue();
            var face = entry.getKey();

            byte[] next = borderTemp[face.ordinal()];
            byte[] current = getBorderPropagation(face);

            if (!compareBorders(next, current)) {
                toUpdate.add(neighbor);
            }
        }

        this.bordersPropagationSwap = combineBorders(bordersPropagation, borderTemp);
        this.toUpdateSet = toUpdate;
        return this;
    }

    private byte[][] combineBorders(byte[][] b1, byte[][] b2) {
        if (b1 == null) return b2;

        byte[][] newBorder = new byte[FACES.length][];
        Arrays.setAll(newBorder, i -> new byte[SIDE_LENGTH]);
        for (int i = 0; i < FACES.length; i++) {
            newBorder[i] = combineBorders(b1[i], b2[i]);
        }
        return newBorder;
    }

    private byte[] bake(byte[] content1, byte[] content2) {
        if (content1 == null && content2 == null) return new byte[16 * 16 * 16 / 2];

        if (content1 == null) return content2;
        if (content2 == null) return content1;

        byte[] lightMax = new byte[16 * 16 * 16 / 2];
        for (int i = 0; i < content1.length; i++) {
            // Lower
            byte l1 = (byte) (content1[i] & 0x0F);
            byte l2 = (byte) (content2[i] & 0x0F);

            // Upper
            byte u1 = (byte) ((content1[i] >> 4) & 0x0F);
            byte u2 = (byte) ((content2[i] >> 4) & 0x0F);

            byte lower = (byte) Math.max(l1, l2);
            byte upper = (byte) Math.max(u1, u2);

            lightMax[i] = (byte) (lower | (upper << 4));
        }
        return lightMax;
    }

    @Override
    public byte[] getBorderPropagation(BlockFace face) {
        if (!isValid) clearCache();

        if (borders == null && bordersPropagation == null) return new byte[SIDE_LENGTH];
        if (borders == null) return bordersPropagation[face.ordinal()];
        if (bordersPropagation == null) return borders[face.ordinal()];

        return combineBorders(bordersPropagation[face.ordinal()], borders[face.ordinal()]);
    }

    @Override
    public void invalidatePropagation() {
        this.isValid = false;
        this.bordersPropagation = null;
        this.contentPropagation = null;
        this.baked = null;
    }

    private void freePropagation() {
        this.baked = bake(content, contentPropagation);
        this.content = null;
        this.borders = null;
        this.bordersPropagation = null;
        this.contentPropagation = null;
        this.isValidBase = false;
    }

    @Override
    public int getLevel(int x, int y, int z) {
        var array = array();
        int index = x | (z << 4) | (y << 8);
        return BlockLightCompute.getLight(array, index);
    }

    private byte[] combineBorders(byte[] b1, byte[] b2) {
        byte[] newBorder = new byte[SIDE_LENGTH];
        for (int i = 0; i < newBorder.length; i++) {
            var previous = b2[i];
            var current = b1[i];
            newBorder[i] = (byte) Math.max(previous, current);
        }
        return newBorder;
    }
}
