import os, sys, struct, glob, io

# R5D1 packer: keyframe (frame 0) + delta frames using skip/copy runs on 16-bit words.
# Format:
#   Header: b'R5D1' + u16 width + u16 height + u32 frames + u8 endian(1=BE) + 7 bytes reserved
#   For each frame:
#     u8 type (0=key,1=delta), u32 payload_len, payload bytes
#   Key payload: raw RGB565BE of size width*height*2
#   Delta payload: sequence of u16 run headers and literals:
#       header with high bit=0 -> skip N words unchanged (0..0x7FFF)
#       header with high bit=1 -> copy K words that follow (1..0x8000)  (len = (header&amp;0x7FFF)+1)
# The decoder applies runs over previous frame buffer.

WIDTH, HEIGHT = 368, 448
WORDS = WIDTH*HEIGHT
ENDIAN_BE = 1

src_dir, out_path = sys.argv[1], sys.argv[2]
frames = sorted(glob.glob(os.path.join(src_dir, 'frame_*.raw')))
if not frames:
    print('no frames', file=sys.stderr)
    sys.exit(2)

with open(out_path, 'wb') as out:
    # Header
    out.write(b'R5D1')
    out.write(struct.pack('>HHI', WIDTH, HEIGHT, len(frames)))  # BE header
    out.write(struct.pack('>B', ENDIAN_BE))
    out.write(b'\x00'*7)

    # Load first frame (key)
    with open(frames[0], 'rb') as f0:
        key = f0.read()
    if len(key) != WORDS*2:
        raise SystemExit(f'frame0 size mismatch: {len(key)} != {WORDS*2}')

    # Write key frame
    out.write(struct.pack('>BI', 0, len(key)))  # type=0, payload_len
    out.write(key)

    prev = key

    # Process delta frames
    for idx, fp in enumerate(frames[1:], start=1):
        with open(fp, 'rb') as f:
            cur = f.read()
        if len(cur) != WORDS*2:
            raise SystemExit(f'frame{idx} size mismatch: {len(cur)}')

        # Work on 16-bit words (big-endian)
        # We will compare as 2-byte slices to avoid endian conversions.
        buf = io.BytesIO()
        i = 0
        # Fast path: if identical, write a single skip of all words (saves payload)
        if cur == prev:
            remaining = WORDS
            while remaining > 0:
                n = min(0x7FFF, remaining)
                buf.write(struct.pack('>H', n))
                remaining -= n
        else:
            # General skip/copy runs
            while i < WORDS:
                # SKIP unchanged
                skip = 0
                while i < WORDS and prev[2*i:2*i+2] == cur[2*i:2*i+2] and skip < 0x7FFF:
                    skip += 1
                    i += 1
                buf.write(struct.pack('>H', skip))  # high bit=0
                if i >= WORDS:
                    break
                # COPY changed
                start = i
                cnt = 0
                while i < WORDS and prev[2*i:2*i+2] != cur[2*i:2*i+2] and cnt < 0x8000:
                    cnt += 1
                    i += 1
                header = 0x8000 | (cnt - 1)
                buf.write(struct.pack('>H', header))
                buf.write(cur[2*start:2*i])

        payload = buf.getvalue()
        out.write(struct.pack('>BI', 1, len(payload)))  # type=1 delta
        out.write(payload)
        prev = cur
