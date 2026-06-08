def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def box_iou(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    ax2 = ax1 + aw
    ay2 = ay1 + ah
    bx2 = bx1 + bw
    by2 = by1 + bh
    inter_w = min(ax2, bx2) - max(ax1, bx1)
    inter_h = min(ay2, by2) - max(ay1, by1)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0
    inter = inter_w * inter_h
    union = aw * ah + bw * bh - inter
    if union <= 0:
        return 0.0
    return inter / union


def center_dist2(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    acx = ax1 + aw // 2
    acy = ay1 + ah // 2
    bcx = bx1 + bw // 2
    bcy = by1 + bh // 2
    dx = acx - bcx
    dy = acy - bcy
    return dx * dx + dy * dy


def mat_solve_8x8(A, B):
    n = 8
    m = [[A[i][j] for j in range(n)] for i in range(n)]
    b = [B[i] for i in range(n)]
    for col in range(n):
        max_val, max_row = abs(m[col][col]), col
        for row in range(col + 1, n):
            if abs(m[row][col]) > max_val:
                max_val, max_row = abs(m[row][col]), row
        if max_row != col:
            m[col], m[max_row] = m[max_row], m[col]
            b[col], b[max_row] = b[max_row], b[col]
        if abs(m[col][col]) < 1e-10:
            return None
        for row in range(col + 1, n):
            factor = m[row][col] / m[col][col]
            for j in range(col, n):
                m[row][j] -= factor * m[col][j]
            b[row] -= factor * b[col]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = b[i]
        for j in range(i + 1, n):
            x[i] -= m[i][j] * x[j]
        x[i] /= m[i][i]
    return x


def calc_homography(pixels, world):
    A, B = [], []
    for i in range(4):
        px, py = float(pixels[i][0]), float(pixels[i][1])
        wx, wy = float(world[i][0]), float(world[i][1])
        A.append([px, py, 1, 0, 0, 0, -wx * px, -wx * py])
        A.append([0, 0, 0, px, py, 1, -wy * px, -wy * py])
        B.extend([wx, wy])
    h = mat_solve_8x8(A, B)
    if h is None:
        return None
    return [
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0],
    ]


def pixel_to_world(px, py, H):
    px = float(px)
    py = float(py)
    w = H[2][0] * px + H[2][1] * py + H[2][2]
    if abs(w) < 1e-10:
        return (0.0, 0.0)
    X = (H[0][0] * px + H[0][1] * py + H[0][2]) / w
    Y = (H[1][0] * px + H[1][1] * py + H[1][2]) / w
    return (X, Y)


def world_to_pixel(X, Y, H):
    w = H[2][0] * X + H[2][1] * Y + H[2][2]
    if abs(w) < 1e-10:
        return (-1, -1)
    u = (H[0][0] * X + H[0][1] * Y + H[0][2]) / w
    v = (H[1][0] * X + H[1][1] * Y + H[1][2]) / w
    return (int(u), int(v))
