#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices

class CloudFixer:
    def __init__(self):
        rospy.init_node('cloud_fixer_node')
        
        # 1. 订阅原始数据
        self.sub = rospy.Subscriber("/depth_camera/points", PointCloud2, self.callback)
        # 2. 发布矫正后的数据
        self.pub = rospy.Publisher("/cloud_corrected", PointCloud2, queue_size=1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo(">>> 纯净模式: 仅做坐标转换 + 地面过滤，不屏蔽前方任何数据 <<<")

    def callback(self, cloud_msg):
        try:
            # --- 1. 获取 TF (Map -> Camera) ---
            trans = self.tf_buffer.lookup_transform("map", cloud_msg.header.frame_id, rospy.Time(0))
            
            t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            
            mat_t = translation_matrix(t)
            mat_r = quaternion_matrix(q)
            mat_total = concatenate_matrices(mat_t, mat_r)

            # --- 2. 读取原始点云 ---
            points_gen = point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points_gen)
            if not points_list: return
            
            P_cam = np.array(points_list, dtype=np.float32) 

            # --- 3. 坐标转换 (转到 Map 系) ---
            # 补齐次坐标
            ones = np.ones((P_cam.shape[0], 1))
            points_hom = np.hstack((P_cam, ones))
            # 矩阵乘法
            P_map = np.dot(points_hom, mat_total.T)[:, :3] 

            # --- 4. 唯一过滤: 地面切除 ---
            # 只切除高度小于 0.1m 的点 (地板)
            # 没有任何距离限制 (min_dist/max_dist 都不管)
            mask_ground = (P_map[:, 2] > 0.1)
            
            P_final = P_map[mask_ground]

            if len(P_final) == 0: return

            # --- 5. 发布 ---
            header = cloud_msg.header
            header.frame_id = "map" 
            new_cloud = point_cloud2.create_cloud_xyz32(header, P_final)
            self.pub.publish(new_cloud)
            
        except Exception as e:
            pass

if __name__ == '__main__':
    try:
        CloudFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
