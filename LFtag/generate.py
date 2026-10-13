import os
import imageio
import numpy as np

def generate(size, name, data):
    assert size > 1
    tag_id = data
    side_length = (3 + 3 * size) * 2

    tag = np.ones((side_length, side_length, 3), dtype=np.uint8)
    tag.fill(255)

    data_blocks = []

    for i in range(size):
        for j in range(size):
            data_blocks.append((j, i))

    data_blocks.remove((0, 0))
    data_blocks.remove((size-1, 0))

    # add border
    for i in range(side_length):
        tag[i, 0, :] = 0
        tag[i, 1, :] = 0

        tag[i, -1, :] = 0
        tag[i, -2, :] = 0

        tag[0, i, :] = 0
        tag[1, i, :] = 0

        tag[-1, i, :] = 0
        tag[-2, i, :] = 0

    tag[4:8, 4:8, :] = 0
    tag[4:8, -8:-4, :] = 0

    blocks = []
    for pos in data_blocks:
        block_data = data & 0b11
        data >>= 2

        blocks.append(block_data)

        sub_pos = [block_data & 1, (block_data & 2) >> 1]

        x = 4 + pos[1] * 6 + sub_pos[0] * 1
        y = 4 + pos[0] * 6 + sub_pos[1] * 1

        tag[x:x+3, y:y+3, :] = 0
        # tag[x, y, :] = 127
        # tag[x + 2, y, :] = 127
        # tag[x, y + 2, :] = 127
        # tag[x + 2, y + 2, :] = 127
    out_dir = os.path.dirname(name)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    imageio.imwrite(name, tag)
    write_dat(name, size, tag_id, len(data_blocks), side_length, blocks)

def write_dat(name, size, tag_id, num_blocks, side_length, blocks):
    """Write a sidecar .dat file with the marker's metadata next to the PNG.

    The .dat is a human-readable key=value text file describing the tag. It
    records the id explicitly so downstream tools don't have to re-decode the
    pixels (the id is otherwise only recoverable from the image or filename)."""
    dat_name = os.path.splitext(name)[0] + ".dat"
    with open(dat_name, "w") as f:
        f.write("# LFTag marker metadata\n")
        f.write(f"algorithm=LFTag\n")
        f.write(f"size={size}\n")
        f.write(f"id={tag_id}\n")
        f.write(f"data_blocks={num_blocks}\n")
        f.write(f"bits={num_blocks * 2}\n")
        f.write(f"side_length={side_length}\n")
        # per-block 2-bit values, in encoding order (LSB block first)
        f.write("blocks=" + ",".join(str(b) for b in blocks) + "\n")

def generate_class(size, upto = 5000):
    data_blocks = size ** 2 - 2
    total_tags = 4 ** data_blocks
    print(f"generating tag with size {size}, {data_blocks * 2} bits and {total_tags} tags")
    for i in range(min(upto, total_tags)):
        generate(size, f"generated/tag{size}/tag{size}_{i:05}.png", i)


if __name__ == "__main__":
    # generate_class(2)
    # generate_class(3)
    generate_class(4)

    i = 2**28 - 1
    generate(4, f"generated/tag4/tag4_{i:05}.png", i)
