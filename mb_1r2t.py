import serial
import pygame
import math
import time
from enum import Enum

class State(Enum):
    SYNC0 = 0
    SYNC1 = 1
    HEADER = 2
    DATA = 3

class ColorMode(Enum):
    DISTANCE = 0
    INTENSITY = 1
    NEON_CYAN = 2

# --- Pygame Setup ---
pygame.init()
pygame.font.init()
WIDTH, HEIGHT = 1000, 900
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.DOUBLEBUF | pygame.RESIZABLE)
pygame.display.set_caption("MB_1R2T 2D LIDAR Visualizer - Modern TF Telemetry")
clock = pygame.time.Clock()

# Fonts
font_title = pygame.font.Font(None, 26)
font_hud = pygame.font.Font(None, 22)
font_small = pygame.font.Font(None, 18)
font_large = pygame.font.Font(None, 34)

# Color Definitions (Futuristic Dark Palette)
COLOR_BG = (10, 14, 23)
COLOR_GRID = (25, 38, 58)
COLOR_GRID_TEXT = (90, 120, 160)
COLOR_SWEEP = (0, 229, 255)
COLOR_TF_X = (255, 55, 95)      # Red (+X Forward)
COLOR_TF_Y = (0, 230, 135)      # Green (+Y Left)
COLOR_TF_Z = (0, 180, 255)      # Blue/Cyan (+Z Yaw)
COLOR_ORIGIN = (0, 229, 255)
COLOR_PANEL_BG = (16, 23, 38, 210)
COLOR_PANEL_BORDER = (0, 180, 255, 80)
COLOR_TEXT_MAIN = (230, 240, 255)
COLOR_TEXT_MUTED = (120, 150, 190)
COLOR_WARNING = (255, 170, 0)
COLOR_ERROR = (255, 60, 60)

# Viewport / Camera Parameters
scale = 100.0  # pixels per meter (default 1m = 100px)
center_x = WIDTH // 2
center_y = HEIGHT // 2
pan_x = 0.0
pan_y = 0.0
is_panning = False
pan_start_pos = (0, 0)

# Display Options
show_grid = True
show_tf_axis = True
show_hud = True
show_sweep = True
color_mode = ColorMode.DISTANCE
is_paused = False

# Telemetry tracking
current_frame_points = []
render_points = []  # [(world_x, world_y, distance_m, intensity, angle_rad)]
fps = 0.0
points_per_sec = 0
point_count_last_frame = 0
min_dist_m = 0.0
max_dist_m = 0.0
last_scan_angle = 0.0
sweep_angle_rad = 0.0
last_packet_time = time.time()
serial_status_msg = "Initializing..."

# Serial Port Init
PORT_NAME = '/dev/cu.usbserial-1120'
BAUD_RATE = 153600

def open_serial():
    global serial_status_msg
    try:
        s = serial.Serial(
            port=PORT_NAME,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.05
        )
        serial_status_msg = f"Connected to {PORT_NAME}"
        return s
    except Exception as e:
        serial_status_msg = f"Disconnected: {e}"
        return None

com = open_serial()

# --- Helper Functions ---
def world_to_screen(wx, wy):
    """
    Converts World coordinates (meters: +X Forward/Up, +Y Left) to Screen coordinates (pixels).
    ROS TF Convention for 2D LIDAR:
    +X points Forward (Up on screen)
    +Y points Left (Left on screen)
    """
    sx = int(center_x + pan_x - wy * scale)
    sy = int(center_y + pan_y - wx * scale)
    return sx, sy

def screen_to_world(sx, sy):
    """Converts Screen coordinates (pixels) to World coordinates (meters)."""
    wy = (center_x + pan_x - sx) / scale
    wx = (center_y + pan_y - sy) / scale
    return wx, wy

def get_point_color(dist_m, intensity, mode):
    """Generates vibrant color gradients based on distance or intensity."""
    if mode == ColorMode.DISTANCE:
        # Distance heatmap: Cyan (0m) -> Green (2m) -> Yellow (4m) -> Magenta/Red (>6m)
        normalized = min(dist_m / 6.0, 1.0)
        if normalized < 0.33:
            t = normalized / 0.33
            r = int(0 + t * 0)
            g = int(229 + t * 26)
            b = int(255 - t * 120)
        elif normalized < 0.66:
            t = (normalized - 0.33) / 0.33
            r = int(0 + t * 255)
            g = int(255 - t * 30)
            b = int(135 - t * 135)
        else:
            t = (normalized - 0.66) / 0.34
            r = 255
            g = int(225 - t * 175)
            b = int(0 + t * 120)
        return (r, g, b)
    elif mode == ColorMode.INTENSITY:
        # Intensity heatmap: Dark Violet -> Cyan -> Bright Yellow/White
        norm = min(intensity / 255.0, 1.0)
        r = int(20 + norm * 235)
        g = int(10 + norm * 245)
        b = int(80 + norm * 175)
        return (r, g, b)
    else:  # Neon Cyan
        return (0, 229, 255)

def draw_arrow(surface, color, start_pos, end_pos, width=3, arrow_size=10):
    """Draws a line with a directional arrow head."""
    pygame.draw.line(surface, color, start_pos, end_pos, width)
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    angle = math.atan2(dy, dx)
    
    p1 = (end_pos[0] - arrow_size * math.cos(angle - math.pi / 6),
          end_pos[1] - arrow_size * math.sin(angle - math.pi / 6))
    p2 = (end_pos[0] - arrow_size * math.cos(angle + math.pi / 6),
          end_pos[1] - arrow_size * math.sin(angle + math.pi / 6))
    pygame.draw.polygon(surface, color, [end_pos, p1, p2])

# --- Main Application Loop ---
state = State.SYNC0
package_type = 0
package_size = 0
package_start = 0
package_stop = 0
last_angle = 0

running = True
show_disconnected_warning = False

while running:
    # --- Event Handling ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.VIDEORESIZE:
            WIDTH, HEIGHT = event.w, event.h
            screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.DOUBLEBUF | pygame.RESIZABLE)
            center_x = WIDTH // 2
            center_y = HEIGHT // 2
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 or event.button == 2:  # Left or Middle click
                is_panning = True
                pan_start_pos = event.pos
            elif event.button == 4:  # Zoom in
                scale = min(scale * 1.15, 2000.0)
            elif event.button == 5:  # Zoom out
                scale = max(scale / 1.15, 5.0)
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1 or event.button == 2:
                is_panning = False
        elif event.type == pygame.MOUSEMOTION:
            if is_panning:
                dx = event.pos[0] - pan_start_pos[0]
                dy = event.pos[1] - pan_start_pos[1]
                pan_x += dx
                pan_y += dy
                pan_start_pos = event.pos
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:  # Reset view
                pan_x, pan_y = 0.0, 0.0
                scale = 100.0
            elif event.key == pygame.K_g:  # Toggle Grid
                show_grid = not show_grid
            elif event.key == pygame.K_t:  # Toggle TF Axis
                show_tf_axis = not show_tf_axis
            elif event.key == pygame.K_h:  # Toggle HUD
                show_hud = not show_hud
            elif event.key == pygame.K_s:  # Toggle Sweep Line
                show_sweep = not show_sweep
            elif event.key == pygame.K_c:  # Cycle Color Mode
                color_mode = ColorMode((color_mode.value + 1) % len(ColorMode))
            elif event.key == pygame.K_SPACE or event.key == pygame.K_p:  # Pause
                is_paused = not is_paused

    # --- Serial Communication & Protocol Decoding ---
    if com is not None and com.is_open and not is_paused:
        try:
            if state == State.SYNC0:
                sync = com.read(1)
                if len(sync) > 0 and sync[0] == 0xAA:
                    state = State.SYNC1
            
            elif state == State.SYNC1:
                sync = com.read(1)
                if len(sync) > 0 and sync[0] == 0x55:
                    state = State.HEADER
                else:
                    state = State.SYNC0
            
            elif state == State.HEADER:
                header = com.read(8)
                if len(header) == 8:
                    package_type = header[0]
                    package_size = header[1]
                    package_start = (header[3] << 8) | header[2]
                    package_stop = (header[5] << 8) | header[4]
                    state = State.DATA
                else:
                    state = State.SYNC0

            elif state == State.DATA:
                if package_size > 0:
                    data = com.read(package_size * 3)
                    if len(data) == package_size * 3 and not (package_type & 0x01):
                        diff = package_stop - package_start
                        if package_stop < package_start:
                            diff = 0xB400 - package_start + package_stop

                        step = diff / (package_size - 1) if diff > 1 else 0

                        for i in range(package_size):
                            intensity = data[i * 3 + 0]
                            distance = (data[i * 3 + 2] << 8) | data[i * 3 + 1]
                            distance_m = distance / 4000.0  # 0.25mm units to meters

                            angle = (package_start + step * i) % 0xB400
                            angle_rad = (angle / 0xB400) * (math.pi * 2)
                            sweep_angle_rad = angle_rad

                            if 0.05 < distance_m < 15.0:  # Valid range filter
                                world_x = math.cos(angle_rad) * distance_m
                                world_y = math.sin(angle_rad) * distance_m
                                current_frame_points.append((world_x, world_y, distance_m, intensity, angle_rad))

                            # Frame Complete Check
                            if angle_rad < last_angle:
                                render_points = list(current_frame_points)
                                point_count_last_frame = len(render_points)
                                if render_points:
                                    dists = [p[2] for p in render_points]
                                    min_dist_m = min(dists)
                                    max_dist_m = max(dists)
                                current_frame_points = []
                                last_packet_time = time.time()
                            
                            last_angle = angle_rad

                state = State.SYNC0
                show_disconnected_warning = False

        except (serial.SerialException, OSError) as e:
            com = None
            serial_status_msg = f"Connection Lost: {e}"
            show_disconnected_warning = True
    else:
        if com is None and not is_paused:
            # Auto-reconnect attempt every 2 seconds
            if time.time() - last_packet_time > 2.0:
                com = open_serial()
                last_packet_time = time.time()
        show_disconnected_warning = (time.time() - last_packet_time > 2.0)

    # --- Render Canvas ---
    screen.fill(COLOR_BG)
    origin_sx, origin_sy = world_to_screen(0, 0)

    # --- 1. Draw Polar Grid & Range Rings ---
    if show_grid:
        grid_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        # Polar Rays (every 30 degrees)
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            # ROS coordinate angles: 0 rad = +X (Up), pi/2 rad = +Y (Left)
            ray_x = math.cos(rad) * 15.0
            ray_y = math.sin(rad) * 15.0
            end_sx, end_sy = world_to_screen(ray_x, ray_y)
            pygame.draw.line(grid_surface, (25, 38, 58, 120), (origin_sx, origin_sy), (end_sx, end_sy), 1)

            # Degree labels on outer ring
            label_x = math.cos(rad) * (350 / scale)
            label_y = math.sin(rad) * (350 / scale)
            lsx, lsy = world_to_screen(label_x, label_y)
            if 0 <= lsx <= WIDTH and 0 <= lsy <= HEIGHT:
                deg_str = f"{deg}°"
                txt = font_small.render(deg_str, True, COLOR_GRID_TEXT)
                grid_surface.blit(txt, (lsx - txt.get_width() // 2, lsy - txt.get_height() // 2))

        # Concentric Distance Rings (0.5m, 1m, 2m, 3m, 4m, 5m, 6m, 8m, 10m)
        rings_m = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]
        for r_m in rings_m:
            r_px = int(r_m * scale)
            if r_px > 5:
                pygame.draw.circle(grid_surface, (0, 180, 255, 35), (origin_sx, origin_sy), r_px, 1)
                # Distance label
                lbl_text = f"{r_m:.1f}m ({r_m * 3.28084:.1f}ft)"
                lbl_surf = font_small.render(lbl_text, True, COLOR_GRID_TEXT)
                grid_surface.blit(lbl_surf, (origin_sx + 8, origin_sy - r_px - 10))

        screen.blit(grid_surface, (0, 0))

    # --- 2. Draw Radar Laser Sweep Line ---
    if show_sweep:
        sweep_len = 12.0
        sw_x = math.cos(sweep_angle_rad) * sweep_len
        sw_y = math.sin(sweep_angle_rad) * sweep_len
        sw_sx, sw_sy = world_to_screen(sw_x, sw_y)
        sweep_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.line(sweep_surf, (0, 229, 255, 180), (origin_sx, origin_sy), (sw_sx, sw_sy), 2)
        screen.blit(sweep_surf, (0, 0))

    # --- 3. Draw LIDAR Point Cloud ---
    mouse_pos = pygame.mouse.get_pos()
    hovered_point = None
    min_hover_dist_px = 12.0

    point_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

    for wx, wy, dist_m, intensity, ang_rad in render_points:
        psx, psy = world_to_screen(wx, wy)
        if 0 <= psx <= WIDTH and 0 <= psy <= HEIGHT:
            color = get_point_color(dist_m, intensity, color_mode)
            
            # Point Glow effect
            pygame.draw.circle(point_surface, (*color, 60), (psx, psy), 4)
            pygame.draw.circle(point_surface, (*color, 240), (psx, psy), 2)

            # Check Mouse Hover Inspection
            dist_to_mouse = math.hypot(psx - mouse_pos[0], psy - mouse_pos[1])
            if dist_to_mouse < min_hover_dist_px:
                min_hover_dist_px = dist_to_mouse
                hovered_point = (wx, wy, dist_m, intensity, math.degrees(ang_rad), psx, psy)

    screen.blit(point_surface, (0, 0))

    # --- 4. Draw TF (Transform) Coordinate Frame ---
    if show_tf_axis:
        tf_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        axis_len_px = 70
        
        # ROS Coordinate Standard:
        # +X (Forward) points UP on screen
        # +Y (Left) points LEFT on screen
        x_end_sx = origin_sx
        x_end_sy = origin_sy - axis_len_px
        
        y_end_sx = origin_sx - axis_len_px
        y_end_sy = origin_sy

        # Draw +X Axis (Red Arrow)
        draw_arrow(tf_surface, COLOR_TF_X, (origin_sx, origin_sy), (x_end_sx, x_end_sy), width=3, arrow_size=8)
        lbl_x = font_hud.render("+X (Forward)", True, COLOR_TF_X)
        tf_surface.blit(lbl_x, (x_end_sx + 8, x_end_sy - 5))

        # Draw +Y Axis (Green Arrow)
        draw_arrow(tf_surface, COLOR_TF_Y, (origin_sx, origin_sy), (y_end_sx, y_end_sy), width=3, arrow_size=8)
        lbl_y = font_hud.render("+Y (Left)", True, COLOR_TF_Y)
        tf_surface.blit(lbl_y, (y_end_sx - lbl_y.get_width() - 8, y_end_sy - 15))

        # Draw TF Sensor Origin Marker (Robot / Laser Center)
        pygame.draw.circle(tf_surface, (0, 229, 255, 40), (origin_sx, origin_sy), 14)
        pygame.draw.circle(tf_surface, COLOR_ORIGIN, (origin_sx, origin_sy), 5)
        pygame.draw.circle(tf_surface, (10, 14, 23), (origin_sx, origin_sy), 2)

        # TF Frame Name Label
        lbl_frame = font_small.render("frame_id: laser_frame", True, (0, 229, 255))
        tf_surface.blit(lbl_frame, (origin_sx + 10, origin_sy + 10))

        screen.blit(tf_surface, (0, 0))

    # --- 5. Draw Mouse Hover Measurement HUD ---
    if hovered_point:
        hwx, hwy, hdist, hint, hang, hpsx, hpsy = hovered_point
        # Line from origin to hover point
        pygame.draw.line(screen, (255, 255, 0, 180), (origin_sx, origin_sy), (hpsx, hpsy), 1)
        pygame.draw.circle(screen, (255, 255, 255), (hpsx, hpsy), 6, 2)

        # Tooltip Panel
        tt_text1 = f"Dist: {hdist:.3f} m ({hdist * 3.28084:.2f} ft)"
        tt_text2 = f"Angle: {hang:.1f}° | Int: {hint}"
        tt_text3 = f"X: {hwx:+.3f} m, Y: {hwy:+.3f} m"

        s1 = font_small.render(tt_text1, True, (255, 255, 255))
        s2 = font_small.render(tt_text2, True, (0, 229, 255))
        s3 = font_small.render(tt_text3, True, (255, 200, 0))

        tt_w = max(s1.get_width(), s2.get_width(), s3.get_width()) + 20
        tt_h = 65
        tt_x = min(hpsx + 15, WIDTH - tt_w - 10)
        tt_y = min(hpsy - 15, HEIGHT - tt_h - 10)

        tt_bg = pygame.Surface((tt_w, tt_h), pygame.SRCALPHA)
        tt_bg.fill((16, 24, 40, 230))
        pygame.draw.rect(tt_bg, (0, 229, 255, 200), (0, 0, tt_w, tt_h), 1, border_radius=6)
        screen.blit(tt_bg, (tt_x, tt_y))
        screen.blit(s1, (tt_x + 10, tt_y + 8))
        screen.blit(s2, (tt_x + 10, tt_y + 26))
        screen.blit(s3, (tt_x + 10, tt_y + 44))

    # --- 6. Draw Main HUD Telemetry Overlays ---
    if show_hud:
        hud_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        
        # --- Top Left: Telemetry Panel ---
        panel_w, panel_h = 320, 215
        p_surf = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        p_surf.fill(COLOR_PANEL_BG)
        pygame.draw.rect(p_surf, COLOR_PANEL_BORDER, (0, 0, panel_w, panel_h), 1, border_radius=8)

        # Title
        t_title = font_title.render("MB_1R2T TELEMETRY HUD", True, (0, 229, 255))
        p_surf.blit(t_title, (14, 12))
        pygame.draw.line(p_surf, (0, 180, 255, 60), (14, 36), (panel_w - 14, 36), 1)

        # Metrics
        fps = clock.get_fps()
        st_color = (0, 255, 135) if com and com.is_open and not show_disconnected_warning else COLOR_ERROR
        st_text = "LIVE STREAMING" if st_color == (0, 255, 135) else "NO DATA / WAITING"
        if is_paused:
            st_text = "PAUSED [SPACE]"
            st_color = COLOR_WARNING

        lines = [
            ("Status:", st_text, st_color),
            ("Port:", f"{PORT_NAME} @ {BAUD_RATE} bps", COLOR_TEXT_MAIN),
            ("Firmware Type:", f"0x{package_type:02X} (Header ID)", (255, 200, 0)),
            ("FPS / Rate:", f"{fps:.1f} FPS | Scan Rate: {fps/5.0:.1f} Hz", COLOR_TEXT_MAIN),
            ("Points / Frame:", f"{point_count_last_frame} pts", (0, 229, 255)),
            ("Distance Range:", f"{min_dist_m:.2f} m  ->  {max_dist_m:.2f} m", (255, 200, 0)),
            ("Zoom Level:", f"{scale:.1f} px/m (Scale 1:{100.0/scale:.2f})", COLOR_TEXT_MUTED),
            ("Color Mode:", color_mode.name, (0, 230, 135)),
        ]

        y_off = 44
        for label, val, color in lines:
            txt_lbl = font_hud.render(label, True, COLOR_TEXT_MUTED)
            txt_val = font_hud.render(val, True, color)
            p_surf.blit(txt_lbl, (14, y_off))
            p_surf.blit(txt_val, (130, y_off))
            y_off += 20

        hud_surf.blit(p_surf, (15, 15))

        # --- Top Right: Controls Guide Panel ---
        ctrl_w, ctrl_h = 240, 160
        c_surf = pygame.Surface((ctrl_w, ctrl_h), pygame.SRCALPHA)
        c_surf.fill(COLOR_PANEL_BG)
        pygame.draw.rect(c_surf, COLOR_PANEL_BORDER, (0, 0, ctrl_w, ctrl_h), 1, border_radius=8)

        ct_title = font_title.render("CONTROLS & SHORTCUTS", True, (255, 200, 0))
        c_surf.blit(ct_title, (14, 12))
        pygame.draw.line(c_surf, (255, 200, 0, 60), (14, 36), (ctrl_w - 14, 36), 1)

        controls = [
            ("[Scroll]", "Zoom In / Out"),
            ("[Drag Mouse]", "Pan Canvas View"),
            ("[R]", "Reset Pan / Zoom"),
            ("[C]", "Cycle Color Palette"),
            ("[G] / [T]", "Toggle Grid / TF Axis"),
            ("[SPACE] / [P]", "Pause / Resume"),
            ("[H]", "Toggle HUD Display"),
        ]

        cy_off = 42
        for k_text, d_text in controls:
            k_surf = font_small.render(k_text, True, (0, 229, 255))
            d_surf = font_small.render(d_text, True, COLOR_TEXT_MAIN)
            c_surf.blit(k_surf, (14, cy_off))
            c_surf.blit(d_surf, (105, cy_off))
            cy_off += 16

        hud_surf.blit(c_surf, (WIDTH - ctrl_w - 15, 15))
        screen.blit(hud_surf, (0, 0))

    # --- 7. Draw Hardware Diagnostic Warning (If No Serial Data) ---
    if show_disconnected_warning and not is_paused:
        warn_w, warn_h = 520, 160
        w_surf = pygame.Surface((warn_w, warn_h), pygame.SRCALPHA)
        w_surf.fill((35, 12, 18, 235))
        pygame.draw.rect(w_surf, (255, 60, 90, 220), (0, 0, warn_w, warn_h), 2, border_radius=10)

        w_title = font_large.render("⚠️ NO LIDAR SERIAL DATA DETECTED", True, COLOR_ERROR)
        w_surf.blit(w_title, (20, 16))

        w_lines = [
            f"Port: {PORT_NAME} ({serial_status_msg})",
            "1. Verify LIDAR TX -> USB-Serial RX connection.",
            "2. Verify LIDAR 5V & GND power connections.",
            "3. Ensure Motor Enable (M_CTR/DEV_EN) pin is wired to 5V.",
        ]

        wy_off = 54
        for line in w_lines:
            wt = font_hud.render(line, True, (255, 220, 180))
            w_surf.blit(wt, (20, wy_off))
            wy_off += 24

        screen.blit(w_surf, ((WIDTH - warn_w) // 2, HEIGHT - warn_h - 25))

    # Update Display
    pygame.display.flip()
    clock.tick(60)

if com and com.is_open:
    com.close()
pygame.quit()
