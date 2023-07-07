import numpy as np

from scipy.spatial.transform import Rotation

class CalibratedCamera:
    def __init__(self, name : str, R_cam_pinhole : Rotation) -> None:
        self.name = name
        self.R_cam_pinhole = R_cam_pinhole
        self.K = None
        self.D = None

        print(f"Initialise calibrated camera {self.name} of type {self.type_name}")
        print(f"calibrated pinhole is at Rotation:")
        print(R_cam_pinhole.as_matrix())

        # TODO add roll pitch yaw estimation

        roll, pitch, yaw = R_cam_pinhole.as_euler('zxy', degrees=True)

        print("Roll: {:.2f} degrees".format(roll))
        print("Pitch: {:.2f} degrees".format(pitch))
        print("Yaw: {:.2f} degrees".format(yaw))
        

    def set_intrinsics(self, intrinsics : dict, distortion: dict):
        raise RuntimeError("not implemented")

    def undistort(self, K_out):
        raise RuntimeError("not implemented")
    
    def distort(self, image):
        raise RuntimeError("not implemented")
    

class RadTanCamera(CalibratedCamera):
    def __init__(self, name: str, R_cam_pinhole: Rotation) -> None:
        self.type_name = "pinhole-radtan"
        super().__init__(name, R_cam_pinhole)
        

    def set_intrinsics(self, intrinsics: dict, distortion: dict):
        self.K = np.eye(3, dtype=np.float32)
        self.K[0][0] = intrinsics[0]
        self.K[1][1] = intrinsics[1]
        self.K[0][2] = intrinsics[2]
        self.K[1][2] = intrinsics[3]

        print(f"camera {self.name} initialise K to:")
        print(self.K)

        assert len(distortion) == 4
        k1 = distortion[0]
        k2 = distortion[1]
        p1 = distortion[2]
        p2 = distortion[3]
        k3 = 0

        self.D = np.array([k1, k2, p1, p2, k3], dtype=np.float32) # follow OpenCV convention

        print(f"camera {self.name} initialise D to:")
        print(self.D)

        # to initialise map using initUndistortRectifyMap
        self.map1 = None # map_x(u, v) -> map a pixel coordinate from the ideal pinhole image back to the actual sensor image
        self.map2 = None # map_y(u, v)

    def undistort(self, image, K_out, new_size, no_distortion = False):


        # the R here is to be defined as transfromation from unrectified to rectified, hence the inv()
        if no_distortion:
            D = np.zeros_like(self.D)
            self.undistorted_image = cv2.undistort(image, self.K, D, newCameraMatrix=K_out)
        else:
            D = self.D
            self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, D, self.R_cam_pinhole.inv().as_matrix(), K_out, new_size, cv2.CV_16SC2)

            self.undistorted_image = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)

        return self.undistorted_image
    
class Kb4Camera(CalibratedCamera):
    def __init__(self, name: str, R_cam_pinhole: Rotation) -> None:
        self.type_name = "pinhole-kb4"
        super().__init__(name, R_cam_pinhole)

    def set_intrinsics(self, intrinsics: dict, distortion: dict):
        self.K = np.eye(3, dtype=np.float32)
        self.K[0][0] = intrinsics[0]
        self.K[1][1] = intrinsics[1]
        self.K[0][2] = intrinsics[2]
        self.K[1][2] = intrinsics[3]

        print(f"camera {self.name} initialise K to:")
        print(self.K)

        assert len(distortion) == 4
        k1 = distortion[0]
        k2 = distortion[1]
        k3 = distortion[2]
        k4 = distortion[3]

        self.D = np.array([k1, k2, k3, k4], dtype=np.float32) # follow OpenCV convention

        print(f"camera {self.name} initialise D to:")
        print(self.D)

        # to initialise map using initUndistortRectifyMap
        self.map1 = None # map_x(u, v) -> map a pixel coordinate from the ideal pinhole image back to the actual sensor image
        self.map2 = None # map_y(u, v)

    def undistort(self, image, K_out, new_size, no_distortion = False):
        

        # the R here is to be defined as transfromation from unrectified to rectified, hence the inv()
        if no_distortion:
            D = np.zeros_like(self.D)
            self.undistorted_image = cv2.undistort(image, self.K, D, None, K_out)
            # https://answers.opencv.org/question/73817/issues-with-fisheye-camera-undistortion/
            # there is some issue with cv2.fisheye.undistort
            # cv2.imshow("image", image)
            # cv2.imshow("self.undistorted_image", self.undistorted_image)
            # cv2.waitKey(0)
        else:
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, self.R_cam_pinhole.inv().as_matrix(), K_out, new_size, cv2.CV_32FC1)

            self.undistorted_image = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)

        return self.undistorted_image
    
    def distort(self, image, K):

        distorted = distort_image(image, K, self.D, crop_output=False)

        return distorted