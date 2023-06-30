from my_nav_interface.srv import PathFromModel

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from PIL import Image
import torch
from torchvision import transforms
from models.model import AttnNav
from models.encoder import VisionTransformer
from models.decoder import TransformerDecoder
import model_config as CFG


class PathService(Node):

    def __init__(self):
        super().__init__('path_service')
        self.srv = self.create_service(PathFromModel, 'path_from_model', self.path_from_model_callback) 
        rgb_encoder = VisionTransformer(
                            img_size=CFG.img_size,
                            patch_size=CFG.patch_size,
                            input_channels=3,
                            embed_dim=CFG.embed_dim,
                            depth=CFG.depth,
                            num_heads=CFG.num_heads,
                            drop_rate=CFG.drop_rate,
                            attn_drop_rate=CFG.attn_drop_rate,
                            drop_path_rate=CFG.drop_path_rate
                            )

        lidar_encoder = VisionTransformer(
                            img_size=CFG.img_size,
                            patch_size=CFG.patch_size,
                            input_channels=1,
                            embed_dim=CFG.embed_dim,
                            depth=CFG.depth,
                            num_heads=CFG.num_heads,
                            drop_rate=CFG.drop_rate,
                            attn_drop_rate=CFG.attn_drop_rate,
                            drop_path_rate=CFG.drop_path_rate
                            )

        rob_traj_decoder = TransformerDecoder(
                            embed_dim=CFG.embed_dim,
                            depth=CFG.depth,
                            num_heads=CFG.num_heads,
                            drop_rate=CFG.drop_rate,
                            attn_drop_rate=CFG.attn_drop_rate,
                            drop_path_rate=CFG.drop_path_rate,
                            auto_reg=CFG.auto_reg
                            )

        mot_decoder = TransformerDecoder(
                            embed_dim=CFG.embed_dim,
                            depth=CFG.depth,
                            num_heads=CFG.num_heads,
                            drop_rate=CFG.drop_rate,
                            attn_drop_rate=CFG.attn_drop_rate,
                            drop_path_rate=CFG.drop_path_rate,
                            multi=True
                            )
        
        self.model = AttnNav.load_from_checkpoint('./src/socialnav/saved_model/rob_train_3sec_spread_pose04-03-2023-19-23-47.ckpt',
                                     rgb_encoder=rgb_encoder,
                                     lidar_encoder=lidar_encoder,
                                     rob_traj_decoder=rob_traj_decoder,
                                     mot_decoder=mot_decoder,
                                     auto_reg=CFG.auto_reg,
                                    )

    
    def path_from_model_callback(self, request, response):
        init_pose = request.init_pose
        rgb_img = Image.open('./src/socialnav/data/rgb_img.png')
        lidar = Image.open('./src/socialnav/data/lidar.png')

        transform = transforms.Compose([
            transforms.Resize((240,240)),
            transforms.PILToTensor(),
            transforms.ConvertImageDtype(torch.float),
        ])

        rgb_tensor = transform(rgb_img).unsqueeze(dim=0)
        lidar_tensor = transform(lidar).unsqueeze(dim=0)

        rgb_enc_out = self.model.rgb_encoder(rgb_tensor)
        lidar_enc_out = self.model.lidar_endcoder(lidar_tensor)

        enc_output = torch.cat([rgb_enc_out, lidar_enc_out], dim=2)
        gen_seq = torch.zeros((1,7,2), dtype=torch.float32)

        for ts in range(0,6):
            dec_output = self.rob_decoder(enc_output[:,1:], gen_seq[:,:ts+1])
            gen_seq[:,ts+1] = dec_output[:,-1]
        

        response.path.header.stamp = self.get_clock().now().to_msg()

        for i in range(0,7):
            pose = PoseStamped()
            pose.pose.position.x = gen_seq[0,i,0]
            pose.pose.position.y = gen_seq[0,i,1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.header.stamp = self.get_clock().now().to_msg()
            response.path.poses.append(pose)

        return response

def main():
    rclpy.init()
    pathService = PathService()
    rclpy.spin(pathService)
    rclpy.shutdown()


if __name__ == '__main__':
    main()