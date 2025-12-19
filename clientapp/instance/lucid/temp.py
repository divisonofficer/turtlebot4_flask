import numpy as np


data = np.load("tmp/lucid/08_22_19_01/15_39_31_030/post.npz")


depth = data["depth"]


disparity = data["disparity"].astype(np.float32)


focal_length = data["k_left"][0, 0].astype(np.float32)
baseline = np.abs(data["T"][0]).astype(np.float32)
depth_lidar = data["projected_depth"]
print(depth_lidar[:100])
depth_lidar[:, 2] = focal_length * baseline / depth_lidar[:, 2]
depth_lidar = depth_lidar[depth_lidar[:, 2] > 0]

u = depth_lidar[:, 0].astype(np.int32)
v = depth_lidar[:, 1].astype(np.int32)
disparity_lidar = depth_lidar[:, 2]
print(u.shape, v.shape)
disparity_uv = disparity[v, u]
disparity_lidar = disparity_lidar[disparity_uv > 0]
disparity_uv = disparity_uv[disparity_uv > 0]
print(np.sqrt(np.sum((disparity_uv - disparity_lidar) ** 2)) / disparity_uv.shape[0])

print(disparity_uv[200:250])
print(disparity_lidar[200:250])

# print(disparity_lidar.min(), disparity_lidar.max(), np.median(disparity_lidar))

print(depth.min(), depth.max(), np.median(depth))


depth_new = focal_length * baseline / disparity

depth_new[np.isnan(depth_new)] = 0
depth_new[np.isinf(depth_new)] = 0

print(depth_new.min(), depth_new.max(), np.median(depth_new))


print(depth_new.shape)


print(disparity[disparity > 0].min())

print(focal_length * baseline / 0.00830245)
print(focal_length * baseline / 75633.016)
