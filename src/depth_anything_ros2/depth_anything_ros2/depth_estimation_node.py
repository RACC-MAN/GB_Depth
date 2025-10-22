import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from gbdepth_msgs.msg import DepthData

import cv2
import torch

from depth_anything_v2.dpt import DepthAnythingV2

class DepthEstimationNode(Node):
    
    def __init__(self):
        super().__init__('depth_estimation_node')
        self.publisher_ = self.create_publisher(Image, 'topic', self.callback, 10)
        self.image_sub_ = self.create_subscription(Image, )
        self.image_pub_ = self.create_publisher(Image, '', 10)
        self.depth_pub_ = self.create_publisher(DepthData, '', 10)
        self.depth_n_pub_ = self.create_publisher(DepthData, '', 10)
        self.height_pub_ = self.create_publisher(DepthData, '', 10)

        self.SENSOR_DIST_ 
        self.EYE_POSITION_

    def initialize_model(self):
        DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        encoder = 'vitl' # or 'vits', 'vitb', 'vitg'

        self.model = DepthAnythingV2(**model_configs[encoder])
        self.model.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
        self.model = self.model.to(DEVICE).eval()

    
    def callback(self):
        raw_img = cv2.imread('your/image/path')
        depth = self.model.infer_image(raw_img) 

def main(args=None):
    rclpy.init(args=args)

    depth_estimation_node = DepthEstimationNode()

    rclpy.spin(depth_estimation_node)
    depth_estimation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()