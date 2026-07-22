# ======================================================================
# 28-point raw-image ground-coordinate mesh test for OpenART Plus
#
# No checkerboard, lens_corr(), bird-view image, or camera model is used.
# The mesh is calibrated from the red bag's bottom-center pixel and measured
# ground coordinate at 28 positions.
# ======================================================================

import sensor, image, time


CAMERA_GROUND_MESH_PATH = '/sd/camera_ground_mesh.txt'
CAMERA_ROLE = 'master'  # Change to 'slave' for the slave camera.

IMAGE_W = 320
IMAGE_H = 240
MAX_WORLD_X_CM = 250.0
MAX_WORLD_Y_CM = 164.0
REQUIRED_NEAR_Y_CM = 6.0
SOFTWARE_HMIRROR = True
SENSOR_VFLIP = True
WB_GAINS = (92.00, 64.00, 101.00)
EXPOSURE_US = 880
GAIN_DB = 0

# Match main.py color ID 2. A complete /sd/color_thr.txt overrides this
# fallback exactly as it does in the production runtime.
RED_BAG_COLOR_ID = 2
RED_BAG_THRESHOLD = (10, 80, 22, 122, -17, 93)
MARKER_MIN_PIXELS = 100
MARKER_MIN_AREA = 100
MARKER_ASPECT_MIN = 0.60
MARKER_ASPECT_MAX = 1.80
MARKER_DENSITY_MIN = 0.40
EDGE_MARKER_ASPECT_MIN = 0.20
EDGE_MARKER_ASPECT_MAX = 4.00
EDGE_MARKER_DENSITY_MIN = 0.15

# In collection mode, set these to the measured position of the current bag.
# X is positive right and Y is positive forward, in centimetres.
EXPECTED_X_CM = None
EXPECTED_Y_CM = None
COLLECTION_POINT_ID = 'R5C1'
COLLECTION_SPLIT = 'fit'  # Use 'verify' only for independent check points.

PRINT_INTERVAL_MS = 250
DRAW_VALIDITY_GRID = False
GRID_STEP_X = 40
GRID_STEP_Y = 30
TRIANGLE_EPSILON = 1e-6
OUTSIDE_DEADBAND_PX = 1.5
HOMOGRAPHY_EPSILON = 1e-6

MESH_INVALID = 0
MESH_VALID = 1
MESH_TOO_NEAR = 2
MESH_TOO_FAR = 3
MESH_LEFT = 4
MESH_RIGHT = 5

# Boundary codes stored by calibrate_ground_camera.py.
BOUNDARY_TO_STATUS = {
    1: MESH_TOO_NEAR,
    2: MESH_TOO_FAR,
    3: MESH_LEFT,
    4: MESH_RIGHT,
}


def _parse_float_list(text, count):
    parts = text.split(',')
    if len(parts) != count:
        return None
    values = []
    for part in parts:
        value = float(part.strip())
        if value != value or value <= -1e9 or value >= 1e9:
            return None
        values.append(value)
    return values


def load_red_bag_threshold(path='/sd/color_thr.txt'):
    try:
        rows = {}
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('exposure_us='):
                    continue
                if line.startswith('ground=') or line.startswith('ground2='):
                    continue
                parts = line.split(',')
                if len(parts) == 7:
                    slot = int(parts[0])
                    values = tuple(int(value) for value in parts[1:])
                elif len(parts) == 6:
                    slot = len(rows) + 1
                    values = tuple(int(value) for value in parts)
                else:
                    continue
                if 1 <= slot <= 5 and len(values) == 6:
                    rows[slot] = values
        if len(rows) == 5:
            return rows[RED_BAG_COLOR_ID]
    except Exception:
        pass
    return RED_BAG_THRESHOLD


def load_ground_mesh(path=CAMERA_GROUND_MESH_PATH):
    try:
        rows = {}
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split('=', 1)
                if len(parts) != 2:
                    return None
                key = parts[0].strip()
                if key in rows:
                    return None
                rows[key] = parts[1].strip()

        if int(rows.get('version', '0')) != 4:
            return None
        if rows.get('model') != 'triangle_mesh':
            return None
        if rows.get('role') != CAMERA_ROLE or rows.get('units') != 'cm':
            return None
        if int(rows.get('image_w', '0')) != IMAGE_W:
            return None
        if int(rows.get('image_h', '0')) != IMAGE_H:
            return None
        if int(rows.get('software_hmirror', '-1')) != (1 if SOFTWARE_HMIRROR else 0):
            return None
        if int(rows.get('sensor_vflip', '-1')) != (1 if SENSOR_VFLIP else 0):
            return None
        if int(rows.get('lens_corr', '-1')) != 0:
            return None
        point_count = int(rows.get('point_count', '0'))
        grid_rows = int(rows.get('grid_rows', '0'))
        grid_columns = int(rows.get('grid_columns', '0'))
        if (grid_rows < 4 or grid_rows > 16 or grid_columns != 4 or
                point_count != grid_rows * grid_columns):
            return None

        max_x_cm = float(rows.get('max_x_cm', '0'))
        max_y_cm = float(rows.get('max_y_cm', '0'))
        if not (max_x_cm > 0.0 and max_x_cm <= 250.0):
            return None
        if abs(max_y_cm - MAX_WORLD_Y_CM) > 1e-6:
            return None
        calibrated_y_min_cm = float(rows.get('calibrated_y_min_cm', '0'))
        calibrated_y_max_cm = float(rows.get('calibrated_y_max_cm', '0'))
        if not (0.0 < calibrated_y_min_cm < calibrated_y_max_cm):
            return None
        if calibrated_y_min_cm > REQUIRED_NEAR_Y_CM + 1e-6:
            return None
        if abs(calibrated_y_max_cm - MAX_WORLD_Y_CM) > 1e-6:
            return None
        triangle_count = int(rows.get('triangle_count', '0'))
        expected_triangle_count = (grid_rows - 1) * (grid_columns - 1) * 2
        if triangle_count != expected_triangle_count or triangle_count > 96:
            return None

        triangles = []
        orientation_sign = 0
        for index in range(triangle_count):
            values = _parse_float_list(rows.get('triangle{}'.format(index), ''), 12)
            if values is None:
                return None
            u0, v0, x0, y0 = values[0], values[1], values[2], values[3]
            u1, v1, x1, y1 = values[4], values[5], values[6], values[7]
            u2, v2, x2, y2 = values[8], values[9], values[10], values[11]
            if (u0 < 0.0 or u0 >= IMAGE_W or v0 < 0.0 or v0 >= IMAGE_H or
                    u1 < 0.0 or u1 >= IMAGE_W or v1 < 0.0 or v1 >= IMAGE_H or
                    u2 < 0.0 or u2 >= IMAGE_W or v2 < 0.0 or v2 >= IMAGE_H):
                return None
            if (abs(x0) > max_x_cm or abs(x1) > max_x_cm or abs(x2) > max_x_cm or
                    y0 <= 0.0 or y0 > max_y_cm or
                    y1 <= 0.0 or y1 > max_y_cm or
                    y2 <= 0.0 or y2 > max_y_cm):
                return None

            denominator = ((v1 - v2)*(u0 - u2) +
                           (u2 - u1)*(v0 - v2))
            if not (denominator < -1.0 or denominator > 1.0):
                return None
            world_area2 = ((x1 - x0)*(y2 - y0) -
                           (x2 - x0)*(y1 - y0))
            if not (world_area2 < -1e-4 or world_area2 > 1e-4):
                return None
            sign = 1 if denominator * world_area2 > 0.0 else -1
            if orientation_sign == 0:
                orientation_sign = sign
            elif sign != orientation_sign:
                return None
            triangles.append(tuple(values) + (1.0 / denominator,))
        boundary_count = int(rows.get('boundary_count', '0'))
        expected_boundary_count = 2 * (grid_rows - 1) + 2 * (grid_columns - 1)
        if boundary_count != expected_boundary_count:
            return None
        boundaries = []
        boundary_status_counts = {1: 0, 2: 0, 3: 0, 4: 0}
        near_v_max = -1.0
        for index in range(boundary_count):
            values = _parse_float_list(rows.get('boundary{}'.format(index), ''), 5)
            if values is None:
                return None
            boundary_code = int(values[0])
            if values[0] != boundary_code or boundary_code not in BOUNDARY_TO_STATUS:
                return None
            u0, v0, u1, v1 = values[1], values[2], values[3], values[4]
            if (u0 < 0.0 or u0 >= IMAGE_W or v0 < 0.0 or v0 >= IMAGE_H or
                    u1 < 0.0 or u1 >= IMAGE_W or v1 < 0.0 or v1 >= IMAGE_H):
                return None
            dx = u1 - u0
            dy = v1 - v0
            length2 = dx*dx + dy*dy
            if length2 <= 1.0:
                return None
            boundaries.append((BOUNDARY_TO_STATUS[boundary_code],
                               u0, v0, u1, v1, length2))
            boundary_status_counts[boundary_code] += 1
            if boundary_code == 1:
                near_v_max = max(near_v_max, v0, v1)
        if (boundary_status_counts[1] != grid_columns - 1 or
                boundary_status_counts[2] != grid_columns - 1 or
                boundary_status_counts[3] != grid_rows - 1 or
                boundary_status_counts[4] != grid_rows - 1):
            return None
        if rows.get('fallback_model') != 'homography':
            return None
        fallback_h = _parse_float_list(rows.get('fallback_h', ''), 9)
        if fallback_h is None:
            return None
        determinant = (
            fallback_h[0] * (fallback_h[4]*fallback_h[8] - fallback_h[5]*fallback_h[7]) -
            fallback_h[1] * (fallback_h[3]*fallback_h[8] - fallback_h[5]*fallback_h[6]) +
            fallback_h[2] * (fallback_h[3]*fallback_h[7] - fallback_h[4]*fallback_h[6])
        )
        if -HOMOGRAPHY_EPSILON < determinant < HOMOGRAPHY_EPSILON:
            return None
        return {
            'triangles': triangles,
            'boundaries': boundaries,
            'near_y_cm': calibrated_y_min_cm,
            'far_y_cm': calibrated_y_max_cm,
            'point_count': point_count,
            'grid_rows': grid_rows,
            'near_v_max': near_v_max,
            'fallback_h': tuple(fallback_h),
        }
    except Exception:
        return None


def mesh_pixel_to_ground(mesh, u, v, out):
    if mesh is None or u < 0.0 or u >= IMAGE_W or v < 0.0 or v >= IMAGE_H:
        return MESH_INVALID
    for triangle in mesh['triangles']:
        u0, v0, x0, y0 = triangle[0], triangle[1], triangle[2], triangle[3]
        u1, v1, x1, y1 = triangle[4], triangle[5], triangle[6], triangle[7]
        u2, v2, x2, y2 = triangle[8], triangle[9], triangle[10], triangle[11]
        inverse_denominator = triangle[12]
        a = ((v1 - v2)*(u - u2) + (u2 - u1)*(v - v2)) * inverse_denominator
        b = ((v2 - v0)*(u - u2) + (u0 - u2)*(v - v2)) * inverse_denominator
        c = 1.0 - a - b
        if a >= -TRIANGLE_EPSILON and b >= -TRIANGLE_EPSILON and c >= -TRIANGLE_EPSILON:
            out[0] = a*x0 + b*x1 + c*x2
            out[1] = a*y0 + b*y1 + c*y2
            if out[1] > MAX_WORLD_Y_CM:
                out[1] = MAX_WORLD_Y_CM
            if out[1] <= 0.0:
                return MESH_INVALID
            return MESH_VALID
    return classify_outside_mesh(mesh, u, v)


def nearest_mesh_boundary(mesh, u, v):
    best_status = MESH_INVALID
    best_u = 0.0
    best_v = 0.0
    best_distance2 = 1e30
    for edge in mesh['boundaries']:
        status, u0, v0, u1, v1, length2 = edge
        du = u1 - u0
        dv = v1 - v0
        position = ((u - u0)*du + (v - v0)*dv) / length2
        if position < 0.0:
            position = 0.0
        elif position > 1.0:
            position = 1.0
        nearest_u = u0 + position*du
        nearest_v = v0 + position*dv
        delta_u = u - nearest_u
        delta_v = v - nearest_v
        distance2 = delta_u*delta_u + delta_v*delta_v
        if distance2 < best_distance2:
            best_distance2 = distance2
            best_status = status
            best_u = nearest_u
            best_v = nearest_v
    return (best_status, best_u, best_v)


def classify_outside_mesh(mesh, u, v):
    nearest = nearest_mesh_boundary(mesh, u, v)
    # A contact point below the calibrated near edge is always near. This
    # prevents bottom-of-frame targets from ever taking the far clamp.
    if v > mesh['near_v_max'] + OUTSIDE_DEADBAND_PX:
        return MESH_TOO_NEAR
    return nearest[0]


def mesh_status_name(status):
    if status == MESH_TOO_NEAR:
        return 'TOO_NEAR'
    if status == MESH_TOO_FAR:
        return 'TOO_FAR'
    if status == MESH_LEFT:
        return 'OUT_LEFT'
    if status == MESH_RIGHT:
        return 'OUT_RIGHT'
    return 'INVALID'


def homography_pixel_to_ground(h, u, v):
    denominator = h[6]*u + h[7]*v + h[8]
    if -HOMOGRAPHY_EPSILON < denominator < HOMOGRAPHY_EPSILON:
        return None
    x = (h[0]*u + h[1]*v + h[2]) / denominator
    y = (h[3]*u + h[4]*v + h[5]) / denominator
    if x != x or y != y:
        return None
    return (x, y)


def far_limit_x(h, u):
    far_y = MAX_WORLD_Y_CM
    v_denominator = h[4] - far_y*h[7]
    if -HOMOGRAPHY_EPSILON < v_denominator < HOMOGRAPHY_EPSILON:
        return None
    far_v = (far_y*(h[6]*u + h[8]) - (h[3]*u + h[5])) / v_denominator
    projected = homography_pixel_to_ground(h, u, far_v)
    if projected is None:
        return None
    return projected[0]


def fallback_pixel_to_ground(mesh, u, v, mesh_status, out):
    h = mesh['fallback_h']
    nearest_status, boundary_u, boundary_v = nearest_mesh_boundary(mesh, u, v)
    boundary_world = [0.0, 0.0]
    boundary_valid = mesh_pixel_to_ground(
        mesh, boundary_u, boundary_v, boundary_world) == MESH_VALID
    boundary_h = homography_pixel_to_ground(h, boundary_u, boundary_v)
    correction_x = 0.0
    correction_y = 0.0
    if boundary_valid and boundary_h is not None:
        correction_x = boundary_world[0] - boundary_h[0]
        correction_y = boundary_world[1] - boundary_h[1]

    use_far_limit = mesh_status == MESH_TOO_FAR
    projected = homography_pixel_to_ground(h, u, v)
    if projected is not None:
        x = projected[0] + correction_x
        y = projected[1] + correction_y
        if (mesh_status != MESH_TOO_FAR and
                x >= -MAX_WORLD_X_CM and x <= MAX_WORLD_X_CM and
                y > 0.0 and y <= MAX_WORLD_Y_CM):
            out[0] = x
            out[1] = y
            return True
        if y <= 0.0 or y > MAX_WORLD_Y_CM:
            use_far_limit = True

    if mesh_status == MESH_TOO_NEAR or not use_far_limit:
        return False

    # Pixels above the calibrated far edge are projected onto the Y=164 cm
    # limit along the same image column, avoiding the homography horizon.
    far_x = far_limit_x(h, u)
    if far_x is None:
        return False
    if boundary_valid:
        boundary_far_x = far_limit_x(h, boundary_u)
        if nearest_status == MESH_TOO_FAR and boundary_far_x is not None:
            far_x += boundary_world[0] - boundary_far_x
        else:
            far_x += correction_x
    if far_x < -MAX_WORLD_X_CM or far_x > MAX_WORLD_X_CM:
        return False
    out[0] = far_x
    out[1] = MAX_WORLD_Y_CM
    return True


def largest_red_bag(img, threshold):
    blobs = img.find_blobs([threshold],
                           pixels_threshold=MARKER_MIN_PIXELS,
                           area_threshold=MARKER_MIN_AREA,
                           merge=True)
    best = None
    best_priority = 0
    for blob in blobs:
        if blob.w() <= 0 or blob.h() <= 0:
            continue
        aspect = blob.w() / blob.h()
        density = blob.density()
        strict = (aspect >= MARKER_ASPECT_MIN and aspect <= MARKER_ASPECT_MAX and
                  density >= MARKER_DENSITY_MIN)
        touches_edge = (blob.x() <= 0 or blob.y() <= 0 or
                        blob.x() + blob.w() >= IMAGE_W or
                        blob.y() + blob.h() >= IMAGE_H)
        relaxed_edge = (touches_edge and
                        aspect >= EDGE_MARKER_ASPECT_MIN and
                        aspect <= EDGE_MARKER_ASPECT_MAX and
                        density >= EDGE_MARKER_DENSITY_MIN)
        if strict:
            priority = 2
        elif relaxed_edge:
            priority = 1
        else:
            continue
        if (best is None or priority > best_priority or
                (priority == best_priority and blob.pixels() > best.pixels())):
            best = blob
            best_priority = priority
    return best


def draw_validity_grid(img, mesh, scratch):
    for py in range(GRID_STEP_Y - 1, IMAGE_H, GRID_STEP_Y):
        for px in range(GRID_STEP_X - 1, IMAGE_W, GRID_STEP_X):
            status = mesh_pixel_to_ground(mesh, px, py, scratch)
            color = (0, 160, 0) if status == MESH_VALID else (160, 0, 0)
            img.draw_circle(px, py, 2, color=color, fill=True)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_hmirror(False)
sensor.set_vflip(SENSOR_VFLIP)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
sensor.set_auto_gain(False, gain_db=GAIN_DB)
sensor.skip_frames(time=500)

mesh = load_ground_mesh()
red_bag_threshold = load_red_bag_threshold()
clock = time.clock()
last_print = time.ticks_ms()
world = [0.0, 0.0]
grid_scratch = [0.0, 0.0]

if mesh is None:
    print('MESH MISSING: red-bag collection mode')
    print('CSV columns: point_id,u,v,Xcm,Ycm,split')
else:
    print('MESH OK: {} points / {} triangles, Y={:.1f}..{:.1f}cm, lens_corr disabled'.format(
          mesh['point_count'], len(mesh['triangles']),
          mesh['near_y_cm'], mesh['far_y_cm']))

while True:
    clock.tick()
    img = sensor.snapshot()
    if SOFTWARE_HMIRROR:
        img = img.replace(hmirror=True)

    blob = largest_red_bag(img, red_bag_threshold)
    if blob is None:
        img.draw_string(2, 2, 'NO RED BAG', color=(255, 255, 0), scale=2)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
            last_print = now
            print('RED_BAG_WORLD status=NOT_FOUND reason=NO_RED_BAG fps={:.1f}'.format(
                  clock.fps()))
        continue

    contact_x = blob.x() + blob.w() / 2.0
    contact_y = blob.y() + blob.h() - 0.5
    img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
    img.draw_cross(int(contact_x), int(contact_y), color=(255, 255, 0), size=7, thickness=2)
    now = time.ticks_ms()
    blob_clipped = (blob.x() <= 0 or blob.y() <= 0 or
                    blob.x() + blob.w() >= IMAGE_W or
                    blob.y() + blob.h() >= IMAGE_H)

    if mesh is None:
        if blob_clipped:
            img.draw_string(2, 2, 'CLIPPED SAMPLE', color=(255, 0, 0), scale=1)
            if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
                last_print = now
                print('collection=REJECTED reason=BLOB_CLIPPED '
                      'pixel=({:.1f},{:.1f})'.format(contact_x, contact_y))
            continue
        img.draw_string(2, 2, 'PIXEL:{:.1f},{:.1f}'.format(contact_x, contact_y),
                        color=(255, 128, 0), scale=1)
        if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
            last_print = now
            if EXPECTED_X_CM is not None and EXPECTED_Y_CM is not None:
                print('{},{:.3f},{:.3f},{:.3f},{:.3f},{}'.format(
                      COLLECTION_POINT_ID, contact_x, contact_y,
                      EXPECTED_X_CM, EXPECTED_Y_CM, COLLECTION_SPLIT))
            else:
                print('pixel=({:.3f},{:.3f}); set EXPECTED_X_CM/EXPECTED_Y_CM'.format(
                      contact_x, contact_y))
        continue

    if DRAW_VALIDITY_GRID:
        draw_validity_grid(img, mesh, grid_scratch)
    status = mesh_pixel_to_ground(mesh, contact_x, contact_y, world)
    coordinate_source = 'MESH'
    fallback_reason = ''
    if status != MESH_VALID:
        status_name = mesh_status_name(status)
        if fallback_pixel_to_ground(mesh, contact_x, contact_y, status, world):
            coordinate_source = 'HOMOGRAPHY_FALLBACK'
            fallback_reason = ' mesh_reason={}'.format(status_name)
        else:
            img.draw_string(2, 2, 'NOT FOUND', color=(255, 0, 0), scale=2)
            if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
                last_print = now
                print('RED_BAG_WORLD status=NOT_FOUND reason={} '
                      'pixel=({:.1f},{:.1f}) fps={:.1f}'.format(
                      status_name, contact_x, contact_y, clock.fps()))
            continue

    prefix = ''
    if blob_clipped:
        prefix += 'C '
    if coordinate_source == 'HOMOGRAPHY_FALLBACK':
        prefix += 'F '
    img.draw_string(2, 2, '{}X:{:.1f} Y:{:.1f}cm'.format(
                    prefix, world[0], world[1]), color=(0, 255, 0), scale=1)
    error_text = ''
    if EXPECTED_X_CM is not None and EXPECTED_Y_CM is not None:
        error_x = world[0] - EXPECTED_X_CM
        error_y = world[1] - EXPECTED_Y_CM
        error_cm = (error_x*error_x + error_y*error_y) ** 0.5
        error_text = ' err={:.2f}cm'.format(error_cm)
        img.draw_string(2, 14, 'ERR:{:.2f}cm'.format(error_cm),
                        color=(255, 128, 0), scale=1)
    if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
        last_print = now
        quality_text = ' quality=CLIPPED_ESTIMATE' if blob_clipped else ''
        print('RED_BAG_WORLD X={:.3f}cm Y={:.3f}cm status=VALID source={} '
              'pixel=({:.1f},{:.1f}){}{}{} fps={:.1f}'.format(
              world[0], world[1], coordinate_source, contact_x, contact_y,
              fallback_reason, quality_text, error_text, clock.fps()))
