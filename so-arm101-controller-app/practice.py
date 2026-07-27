import numpy as np
# T = np.array([[0, -1, 0],
#             [1, 0, 0],
#             [0, 0, 1]])

# R = T[:3, :3]
# p = T[:3, 3]

# print(R)
# print(p)

T = np.array([[0, -1, 0, 1], [1.0, 0, 2], [0, 0, 1, 0], [0, 0, 0, 1]])
q = np.array([3, 0, 0, 1])
q2 = (T@q)[:3]

print(q)
print(q2)
