from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
import rclpy
from rclpy.node import Node
import gi
import numpy as np
from sensor_msgs.msg import CameraInfo

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class BluerovRosCamera(Node):
    def __init__(self):
        super().__init__('bluerov_ros_camera')

        #------------------- Declare Parameters -------------------#
        self.declare_parameter('udp_port', 5600)
        self.udp_port = self.get_parameter('udp_port').value

        self.declare_parameter('utimer', 1/60)
        self.timer_period = self.get_parameter('utimer').value

        self.declare_parameter('topic_raw', '/image')
        self.topic_raw = self.get_parameter('topic_raw').value

        self.declare_parameter('topic_compressed', '/image/compressed')
        self.topic_compressed = self.get_parameter('topic_compressed').value

        self.declare_parameter('publish_raw', False)
        self.publish_raw = self.get_parameter('publish_raw').value

        self.declare_parameter('publish_compressed', True)
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.video = Video(self.udp_port)

        #------------------- Publishers -------------------#
        if self.publish_compressed:
            self.publisher_compressed = self.create_publisher(CompressedImage, self.topic_compressed, 10)

        if self.publish_raw:
            self.publisher_raw = self.create_publisher(Image, self.topic_raw, 10)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        #------------------- CV Bridge -------------------#
        self.bridge = CvBridge()

        # ------------------- Camera Info -------------------#
        self.publisher_info = self.create_publisher( CameraInfo, self.topic_raw + "/camera_info", 10)

    def timer_callback(self):
        if self.video._frame is None: return

        frame = self.video._frame

        if self.publish_compressed:
            msg_compressed = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.publisher_compressed.publish(msg_compressed)

        if self.publish_raw:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_raw.publish(msg)
        
        self.publisher_info.publish(self.camera_info_msg)

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    bluerov_ros_camera = BluerovRosCamera()
    rclpy.spin(bluerov_ros_camera)
    bluerov_ros_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def main(args=None):
    rclpy.init(args=args)
    bluerov_ros_camera = BluerovRosCamera()
    rclpy.spin(bluerov_ros_camera)
    bluerov_ros_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
