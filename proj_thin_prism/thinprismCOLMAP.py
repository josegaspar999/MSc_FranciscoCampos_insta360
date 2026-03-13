import pycolmap
import numpy as np

params = np.array([
852.13705237092313, 850.17193586488634, 1440, 1440,
0.089108710858519902, -0.040448487168936217, -0.00051923858012622775, -0.00032993437201914437,
0.016053928938995387, -0.0038407855887184119, 0.0019045576073914582, 0.0032420175002338338
])

camera = pycolmap.Camera(
    model="THIN_PRISM_FISHEYE",
    width=2880,
    height=2880,
    params=params
)

# Points to project
P = np.array([[10, 25, 15], [50, 13, 20], [200, 400, 300]], dtype=float)

for point in P:
    print(camera.img_from_cam(point))