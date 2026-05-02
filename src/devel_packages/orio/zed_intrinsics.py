import pyzed.sl as sl

# 1. Create InitParameters and set resolution
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD1080  # This sets 1920x1080
init_params.camera_fps = 30                          # Optional: set frame rate

zed = sl.Camera()

# 2. Open the camera with the parameters
status = zed.open(init_params)

if status == sl.ERROR_CODE.SUCCESS:
    # 3. Get the parameters for the resolution we just set
    cam_info = zed.get_camera_information()
    calibration_params = cam_info.camera_configuration.calibration_parameters
    
    # Intrinsic parameters for the left camera
    intrinsic = calibration_params.left_cam
    
    fx = intrinsic.fx
    fy = intrinsic.fy
    cx = intrinsic.cx
    cy = intrinsic.cy
    
    # Confirming the resolution used
    res = cam_info.camera_configuration.resolution
    print(f"Resolution: {res.width}x{res.height}")
    print(f"Focal Length: fx={fx:.2f}, fy={fy:.2f}")
    print(f"Principal Point: cx={cx:.2f}, cy={cy:.2f}")
else:
    print(f"Camera failed to open: {status}")

zed.close()