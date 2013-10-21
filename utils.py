import numpy as np

def rotated_vector(vector, angle):
        return vector.dot(np.array([[np.cos(angle), -np.sin(angle)],
                          [np.sin(angle), np.cos(angle)]]))
def angle_between(v1, v2):
    angle = np.arccos((v1.dot(v2)/np.linalg.norm(v1)) * np.linalg.norm(v2))
    return angle

def codirectional(v1, v2, error=0.0001):
    dot_product = v1.dot(v2)
    magnitudes = np.linalg.norm(v1) * np.linalg.norm(v2)
    return (abs(dot_product - magnitudes) < error)
