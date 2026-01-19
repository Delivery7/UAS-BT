import cv2
import yaml

# ===== CONFIG =====
MAP_IMAGE = "ym_map.pgm"
MAP_YAML = "ym_map.yaml"
# ==================

# Load YAML
with open(MAP_YAML, "r") as f:
    map_yaml = yaml.safe_load(f)

resolution = map_yaml["resolution"]
origin_x, origin_y, _ = map_yaml["origin"]

# Load map image
img = cv2.imread(MAP_IMAGE, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError("Map image not found")

height, width = img.shape

print("===================================")
print("Klik titik MERAH, KUNING, HIJAU")
print("Tekan ESC jika selesai")
print("===================================")

points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Konversi pixel -> map (meter)
        map_x = origin_x + (x * resolution)
        map_y = origin_y + ((height - y) * resolution)

        points.append((map_x, map_y))

        print(f"Klik ke-{len(points)}:")
        print(f"  pixel : ({x}, {y})")
        print(f"  map   : (x={map_x:.3f}, y={map_y:.3f})\n")

cv2.namedWindow("MAP")
cv2.setMouseCallback("MAP", mouse_callback)

while True:
    cv2.imshow("MAP", img)
    key = cv2.waitKey(20)
    if key == 27:  # ESC
        break

cv2.destroyAllWindows()

print("===================================")
labels = ["MERAH", "KUNING", "HIJAU"]
for i, p in enumerate(points):
    label = labels[i] if i < len(labels) else f"POINT_{i+1}"
    print(f"{label}: x={p[0]:.3f}, y={p[1]:.3f}")
print("===================================")
