def map(x: int, in_min: int, in_max: int, out_min: int, out_max: int) -> int:
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def distance_to_camera(focal_length: float, real_object_width: float, object_width: float) -> float:
    return (real_object_width * focal_length) / object_width