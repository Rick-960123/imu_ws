import numpy as np

He = np.hstack((np.identity(3), np.zeros((3, 12))))
He = np.vstack(
    (He, np.concatenate((np.zeros(8), np.array([1]), np.zeros(6)))))
print(He)