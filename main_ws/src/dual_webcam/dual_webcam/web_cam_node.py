#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse
import uvicorn, threading, time

class WebCamNode(Node):
    def __init__(self):
        super().__init__('web_cam_node')
        self.bridge = CvBridge()
        self.img_front = None
        self.img_rear = None
        self.display_mode = False

        # ROS subscriptions
        self.create_subscription(Image, '/front/image_raw', self.front_cb, 10)
        self.create_subscription(Image, '/rear/image_raw', self.rear_cb, 10)
        self.create_subscription(Bool, '/direction_active', self.direction_cb, 10)

        # FastAPI setup
        self.app = FastAPI()
        self.app.get("/")(self.index)
        self.app.get("/video_feed")(self.video_feed)

        # Web server in a separate thread
        threading.Thread(target=self.run_webserver, daemon=True).start()

    def front_cb(self, msg):
        self.img_front = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def rear_cb(self, msg):
        self.img_rear = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def direction_cb(self, msg):
        self.display_mode = msg.data

    def run_webserver(self):
        uvicorn.run(self.app, host='0.0.0.0', port=8000, log_level="warning")

    def gen_frames(self):
        while rclpy.ok():
            if self.img_front is not None and self.img_rear is not None:
                main_img, sub_img = (self.img_front.copy(), self.img_rear) if not self.display_mode else (self.img_rear.copy(), self.img_front)

                h, w = main_img.shape[:2]
                sub_img_small = cv2.resize(sub_img, (w // 4, h // 4))
                main_img[-h//4:, -w//4:] = sub_img_small = sub_img if sub_img is None else cv2.resize(sub_img, (w//4, h//4))

                # 車体のラインを描画
                bottom_left = (int(w * 0.25), h)
                bottom_right = (int(w * 0.75), h)
                center_top = (w // 2, int(h * 0.6))

                cv2.line(main_img, bottom_left, center_top, (0, 255, 0), 3)
                cv2.line(main_img, bottom_right, center_top, (0, 255, 0), 3)

                ret, jpeg = cv2.imencode('.jpg', main_img)
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

            time.sleep(0.05)  # 約20fpsの適切な待機

    async def video_feed(self):
        return StreamingResponse(self.gen_frames(), media_type='multipart/x-mixed-replace; boundary=frame')

    async def index(self):
        return Response(content="""
        <html><body style='margin:0; overflow:hidden;'>
        <img src="/video_feed" style="width:100%;height:100vh;object-fit:contain;">
        <script>
        document.addEventListener('keydown', e=>{ if(e.key==='f')toggleFS(); });
        function toggleFS(){
            document.fullscreenElement?
                document.exitFullscreen():
                document.documentElement.requestFullscreen();
        }
        </script>
        </body>
        </html>
        """, media_type="text/html")

    def run_webserver(self):
        uvicorn.run(self.app, host='0.0.0.0', port=8000, log_level="warning")

    def direction_cb(self, msg):
        self.display_mode = msg.data

    def run_webserver(self):
        uvicorn.run(self.app, host='0.0.0.0', port=8000)

def main(args=None):
    rclpy.init(args=args)
    node = WebCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
