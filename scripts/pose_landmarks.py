#!/usr/bin/env python
"""

"""

import rospy
import cv2
from cv_bridge import (CvBridge)
import mediapipe as mp

from std_msgs.msg import (Bool)
from sensor_msgs.msg import (Image)

from data_collector.msg import (UpperBodyKeypoints)


class PoseLandmarks:
    """
    
    """

    def __init__(
        self,
        node_name,
        camera_name,
        image_rotation,
    ):
        """
        
        """

        # # Private constants:
        self.__CAMERA_NAME = camera_name
        self.__IMAGE_ROTATION = image_rotation

        self.__BRIDGE = CvBridge()

        # Pose landmarks detection:
        self.__MP_DRAWING = mp.solutions.drawing_utils
        self.__MP_POSE = mp.solutions.pose
        self.__POSE = self.__MP_POSE.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # # Public constants:
        self.NODE_NAME = node_name

        # # Private variables:
        self.__cv_image = None
        self.__pose_landmarks_frame = None
        self.__keypoints = UpperBodyKeypoints()

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            'realsense_camera': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            'realsense_camera':
                rospy.Subscriber(
                    f'/{self.__CAMERA_NAME}/color/image_raw',
                    Image,
                    self.__realsense_color_callback,
                ),
        }

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__pose_landmarks_image = rospy.Publisher(
            f'{self.NODE_NAME}/pose_landmarks_image',
            Image,
            queue_size=1,
        )
        self.__upperbody_keypoints = rospy.Publisher(
            f'/physical_workload/upperbody_keypoints',
            UpperBodyKeypoints,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.__CAMERA_NAME}/color/image_raw',
            Image,
            self.__realsense_color_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.

    # # Service handlers:

    # # Topic callbacks:
    def __realsense_color_callback(self, message):
        """

        """

        if not self.__is_initialized:
            self.__dependency_status['realsense_camera'] = True

        self.__cv_image = self.__BRIDGE.imgmsg_to_cv2(
            message,
            desired_encoding='passthrough',
        )

    # # Timer functions:

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __pose_estimation(self):
        """
        The output contains the following normalized coordinates (Landmarks):
        - x and y: Landmark coordinates normalized between 0.0 and 1.0 by the
        image width (x) and height (y). 

        - z: The landmark depth, with the depth
        at the midpoint of the hips as the origin. The smaller the value, the
        closer the landmark is to the camera. The magnitude of z uses roughly
        the same scale as x. 
        
        - visibility: The likelihood of the landmark being
        visible within the image.

        0 - nose
        1 - left eye (inner)
        2 - left eye
        3 - left eye (outer)
        4 - right eye (inner)
        5 - right eye
        6 - right eye (outer)
        7 - left ear
        8 - right ear
        9 - mouth (left)
        10 - mouth (right)
        11 - left shoulder
        12 - right shoulder
        13 - left elbow
        14 - right elbow
        15 - left wrist
        16 - right wrist
        17 - left pinky
        18 - right pinky
        19 - left index
        20 - right index
        21 - left thumb
        22 - right thumb
        23 - left hip
        24 - right hip
        25 - left knee
        26 - right knee
        27 - left ankle
        28 - right ankle
        29 - left heel
        30 - right heel
        31 - left foot index
        32 - right foot index

        https://developers.google.com/mediapipe/solutions/vision/pose_landmarker
        https://developers.google.com/mediapipe/solutions/vision/pose_landmarker/python#image_2
        https://www.hackersrealm.net/post/realtime-human-pose-estimation-using-python
        
        """

        pose_results = None

        # If no image is available.
        if self.__cv_image is None:
            return

        try:
            frame = self.__cv_image

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2RGB,
            )

            # Process the frame for pose detection.
            pose_results = self.__POSE.process(frame)

            # Draw pose landmarks on the frame.
            self.__MP_DRAWING.draw_landmarks(
                frame,
                pose_results.pose_landmarks,
                self.__MP_POSE.POSE_CONNECTIONS,
            )

            # Calculate the rotation matrix.
            height, width = frame.shape[:2]
            rotation_matrix = cv2.getRotationMatrix2D(
                (width / 2, height / 2),
                self.__IMAGE_ROTATION,
                1,
            )

            # Apply the rotation to the image.
            frame = cv2.warpAffine(
                frame,
                rotation_matrix,
                (width, height),
            )

            frame = cv2.flip(frame, 1)

            self.__pose_landmarks_frame = frame

            # If no landmarks were detected.
            if not pose_results.pose_landmarks:
                return

            self.__update_upperbody_keypoints(
                pose_results.pose_landmarks.landmark
            )

        except Exception as e:
            print(e)

    def __publish_landmarks_image(self):
        """
        
        """

        if self.__pose_landmarks_frame is None:
            return

        image_message = self.__BRIDGE.cv2_to_imgmsg(
            self.__pose_landmarks_frame,
            encoding="passthrough",
        )

        self.__pose_landmarks_image.publish(image_message)

    def __update_upperbody_keypoints(self, pose_landmarks):
        """
        
        """

        self.__keypoints.left_shoulder.x = pose_landmarks[11].x
        self.__keypoints.left_shoulder.y = pose_landmarks[11].y
        self.__keypoints.left_shoulder.z = pose_landmarks[11].z

        self.__keypoints.right_shoulder.x = pose_landmarks[12].x
        self.__keypoints.right_shoulder.y = pose_landmarks[12].y
        self.__keypoints.right_shoulder.z = pose_landmarks[12].z

        self.__keypoints.left_elbow.x = pose_landmarks[13].x
        self.__keypoints.left_elbow.y = pose_landmarks[13].y
        self.__keypoints.left_elbow.z = pose_landmarks[13].z

        self.__keypoints.right_elbow.x = pose_landmarks[14].x
        self.__keypoints.right_elbow.y = pose_landmarks[14].y
        self.__keypoints.right_elbow.z = pose_landmarks[14].z

        self.__keypoints.left_wrist.x = pose_landmarks[15].x
        self.__keypoints.left_wrist.y = pose_landmarks[15].y
        self.__keypoints.left_wrist.z = pose_landmarks[15].z

        self.__keypoints.right_wrist.x = pose_landmarks[16].x
        self.__keypoints.right_wrist.y = pose_landmarks[16].y
        self.__keypoints.right_wrist.z = pose_landmarks[16].z

        # Calculate torso and waist coordinates:
        self.__keypoints.chest.x = (
            (pose_landmarks[11].x + pose_landmarks[12].x) / 2
        )
        self.__keypoints.chest.y = (
            (pose_landmarks[11].y + pose_landmarks[12].y) / 2
        )
        self.__keypoints.chest.z = (
            (pose_landmarks[11].z + pose_landmarks[12].z) / 2
        )

        self.__keypoints.waist.x = (
            (pose_landmarks[23].x + pose_landmarks[24].x) / 2
        )
        self.__keypoints.waist.y = (
            (pose_landmarks[23].y + pose_landmarks[24].y) / 2
        )
        self.__keypoints.waist.z = (
            (pose_landmarks[23].z + pose_landmarks[24].z) / 2
        )

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__pose_estimation()

        self.__publish_landmarks_image()
        self.__upperbody_keypoints.publish(self.__keypoints)

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'pose_landmarks',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_name = rospy.get_name()

    camera_name = rospy.get_param(
        param_name=f'{node_name}/camera_name',
        default='camera',
    )
    image_rotation = rospy.get_param(
        param_name=f'{node_name}/image_rotation',
        default=90,
    )

    pose_landmarks = PoseLandmarks(
        node_name=node_name,
        camera_name=camera_name,
        image_rotation=image_rotation,
    )

    rospy.on_shutdown(pose_landmarks.node_shutdown)

    while not rospy.is_shutdown():
        pose_landmarks.main_loop()


if __name__ == '__main__':
    main()