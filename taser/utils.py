import numpy as np

def safe_time(trajectory):
    "Return a time instance that the trajectory is valid for"
    tmin, tmax = trajectory.valid_time

    # Base case: Both finite
    if np.isfinite(tmin) and np.isfinite(tmax):
        t = 0.5 * (tmin + tmax)

    else:
        # At least one is infinite
        # Make sure tmin is not inf or tmax = -inf
        if tmax <= tmin:
            raise ValueError(f"No safe time: tmax <= tmin! ({tmax} <= {tmin})")

        elif np.isfinite(tmin):  # (a, np.inf) -> t >= a OK
            t = tmin + 1
        elif np.isfinite(tmax):  # (-inf, b) -> t < b is OK
            t = tmax - 1
        else:
            t = 42.  # (-inf, inf) means any time is valid, pick something non-zero

    # Sanity check
    if np.isfinite(t):
        return t
    else:
        raise ValueError("No safe time: result was not finite")


def safe_time_span(trajectory, length, *, allow_shorter=False):
    "Return a time span of length that is valid for the trajectory"
    tmin, tmax = trajectory.valid_time

    # Base case: Both finite
    if np.isfinite(tmin) and np.isfinite(tmax):
        max_length = tmax - tmin
        if max_length < length:
            if allow_shorter and max_length > 0:
                result = (tmin, tmax)
            else:
                raise ValueError("No safe time span: trajectory is too short")
        else:
            result = (tmin, tmin + length)

    else:
        # At least one is infinite
        # Make sure tmin is not inf or tmax = -inf
        if tmax <= tmin:
            raise ValueError(f"No safe time span: tmax <= tmin ({tmax} <= {tmin})")
        elif np.isfinite(tmin):  # (a, np.inf) -> t >= a OK
            result = (tmin, tmin + length)
        elif np.isfinite(tmax):  # (-inf, b) -> t < b is OK
            result = (tmax - length, tmax)
        else:
            a = 42.
            result = (a, a + length)

    if np.all(np.isfinite(result)):
        return result
    else:
        raise ValueError("No safe time span: got non-finite result")