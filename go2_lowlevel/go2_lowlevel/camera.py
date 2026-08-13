import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from unitree_go.msg import Go2FrontVideoData
import av, cv2
from cv_bridge import CvBridge
import threading
from collections import deque

class Go2Video(Node):
    def __init__(self):
        super().__init__("go2_video")

        self.camera_sub = self.create_subscription(Go2FrontVideoData, 'frontvideostream', self.camera_cb, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.queue_size = 10
        self.queue = deque(maxlen=self.queue_size)
        self.queue_condition = threading.Condition()

        self.running = True
        self.decoder_thread = threading.Thread(
            target = self.decoder_worker,
            daemon = True,
            name = "h264-decoder",
        )
        self.decoder_thread.start()

    def camera_cb(self, msg):
        if msg.resolution != 360:
            return
        data = bytes(msg.data)
        nal_type = data[4] & 0x1F
        with self.queue_condition:
            if nal_type != 1: # messages are always either nal 1 or nal [5, 7, 8]. if it is not 1, we can safely reset/resync
                self.queue.clear()
            if len(self.queue) < self.queue_size:
                self.queue.append(data)
            self.queue_condition.notify()


    def decoder_worker(self):
        decoder = av.CodecContext.create("h264", "r")

        while True:
            with self.queue_condition:
                while not self.queue and self.running:
                    self.queue_condition.wait()
                if not self.running and not self.queue: 
                    break
                data = self.queue.popleft()

            try:
                packet = av.Packet(data)
                frames = decoder.decode(packet)
                for frame in frames:
                    img = frame.to_ndarray(format="bgr24")
                    img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"H264 decode error: {e}")

    def destroy_node(self):
        with self.queue_condition:
            self.running = False
            self.queue_condition.notify_all()
        if self.decoder_thread.is_alive():
            self.decoder_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2Video()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
