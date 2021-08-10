import pyrealsense2 as rs

# Configure streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.gyro)

# Start streaming
pipeline.start(config)
print(pipeline.get_active_profile().get_streams())


success, frames = pipeline.try_wait_for_frames(timeout_ms=1000)
print(success)
print(frames.get_profile(), frames.get_data_size(), frames.get_data())

pipeline.stop()