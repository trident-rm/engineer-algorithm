import cv2
import pyrealsense2 as rs
import numpy as np
import sys


def list_device_profiles(device):
    profiles = []
    for sensor in device.query_sensors():
        for p in sensor.get_stream_profiles():
            try:
                v = p.as_video_stream_profile()
                profiles.append((p.stream_type(), p.format(), v.width(), v.height(), v.fps()))
            except Exception:
                # non-video profiles (motion, pose, etc.)
                pass
    return profiles


def find_best_video_profile(device, stream_type, desired_w, desired_h, desired_fps, formats=None):
    # formats: iterable of acceptable formats (rs.format.*)
    candidates = []
    for sensor in device.query_sensors():
        for p in sensor.get_stream_profiles():
            try:
                v = p.as_video_stream_profile()
            except Exception:
                continue
            if p.stream_type() != stream_type:
                continue
            if formats and p.format() not in formats:
                continue
            w, h, f = v.width(), v.height(), v.fps()
            # score distance (lower is better)
            score = abs(w - desired_w) + abs(h - desired_h) + abs(f - desired_fps) * 100
            candidates.append((score, w, h, f, p.format()))

    if not candidates:
        return None
    candidates.sort(key=lambda x: x[0])
    # return best (w,h,f,format)
    _, w, h, f, fmt = candidates[0]
    return (w, h, f, fmt)


def test_cam():
    pipeline = rs.pipeline()
    config = rs.config()

    # Create a context object
    context = rs.context()

    devices = context.query_devices()
    if len(devices) == 0:
        print("No RealSense devices found")
        return

    # Use first device
    device = devices[0]
    print("Found device:", device.get_info(rs.camera_info.name))

    # Print available video profiles (diagnostic)
    profiles = list_device_profiles(device)
    print("Supported video profiles (stream, format, w, h, fps):")
    for p in profiles:
        print("  ", p)

    # Desired targets from the user's request
    desired_depth = (1024, 768, 30)
    desired_color = (1920, 1080, 30)

    depth_profile = find_best_video_profile(device, rs.stream.depth, *desired_depth, formats=[rs.format.z16])
    # color cameras often report rgb8 or bgr8; accept both
    color_profile = find_best_video_profile(device, rs.stream.color, *desired_color, formats=[rs.format.bgr8, rs.format.rgb8])

    if depth_profile:
        dw, dh, df, dfmt = depth_profile
        print(f"Selecting depth {dw}x{dh}@{df} fmt={dfmt}")
        config.enable_stream(rs.stream.depth, dw, dh, dfmt, df)
    else:
        print("Requested depth profile not available; falling back to default depth stream")

    if color_profile:
        cw, ch, cf, cfmt = color_profile
        print(f"Selecting color {cw}x{ch}@{cf} fmt={cfmt}")
        config.enable_stream(rs.stream.color, cw, ch, cfmt, cf)
    else:
        print("Requested color profile not available; falling back to default color stream")

    # Try starting pipeline with the requested/selected config. If it fails, try a safe fallback.
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print("pipeline.start(config) failed:", e)
        print("Attempting to start pipeline with default config (no explicit streams)")
        try:
            profile = pipeline.start()
        except Exception as e2:
            print("Failed to start pipeline with default config:", e2)
            return

    try:
        dev = profile.get_device()
        print("Started device:", dev.get_info(rs.camera_info.name))
        # depth scale if available
        try:
            depth_sensor = dev.first_depth_sensor()
            if depth_sensor:
                print("Depth scale:", depth_sensor.get_depth_scale())
        except Exception:
            pass

        while True:
            frames = pipeline.wait_for_frames()
            # frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("RealSense", color_image)
            if depth_frame:
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Depth", depth_colormap)
            key = cv2.waitKey(1)
            if key == 27:
                break
    except KeyboardInterrupt:
        print("Stopping on user interrupt")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def main():
    test_cam()


if __name__ == "__main__":
    main()