import glob
import cv2
import numpy as np

# Checkerboard: 9x6 inner corners (10x7 squares)
pattern_size = (9, 6)
square_size = 1.0  # inches

# Prepare object points
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

images = sorted(glob.glob("camera_frames/VisionPortal-CameraFrameCapture-*.png"))
if not images:
    raise SystemExit("No images found in camera_frames/")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Skip unreadable: {fname}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
    if found:
        objpoints.append(objp)
        imgpoints.append(corners)
    else:
        print(f"Checkerboard not found: {fname}")

if len(objpoints) < 8:
    raise SystemExit("Not enough valid images. Need at least ~8.")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("RMS reprojection error:", ret)
print("Camera matrix (fx, 0, cx / 0, fy, cy):")
print(camera_matrix)
print("Distortion (k1, k2, p1, p2, k3):")
print(dist_coeffs.ravel())
