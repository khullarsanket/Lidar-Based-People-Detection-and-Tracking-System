import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from math import sin, cos
import numpy as np

class MovingObjectLocationDetector(Node):
    def __init__(self):
        super().__init__('moving_object_detector')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.LaserScanCallback, 10)
        self.publisher_moving = self.create_publisher(PointCloud, '/point_cloud_moving', 10)
        self.publisher_stationary = self.create_publisher(PointCloud, '/point_cloud_stationary', 10)
        self.publisher_centroid = self.create_publisher(PointCloud, '/person_locations', 10)
        self.first_scan_data = None
        self.moving_obstacle_threshold = 0.3
        self.header = None
        self.tracks = []
        self.unique_people_count = 0




    def LaserScanCallback(self, scan: LaserScan):
        # self.get_logger().info("Received Laser Scan")

        if self.first_scan_data is None:
            self.first_scan_data = scan.ranges
            self.header = scan.header
            # self.get_logger().info("Stored the ranges of first scan")
            return

        moving_points, stationary_points = self.identify_moving_obstacles(scan)
        
        self.publisher_moving.publish(moving_points)
        self.publisher_stationary.publish(stationary_points)

        
        
        

        identified_clusters, centroids = self.euclidean_clustering(moving_points)
        
        #print(centroids)
        #print(moving_points)
        
        self.publisher_centroid.publish(centroids)
        # self.get_logger().info("Published Centroids Data")
        
    
    def identify_moving_obstacles(self, scan: LaserScan):
        ### Function to identify the moving objects (separating the obstacles from people)
        moving_points = []
        stationary_points = []
        for i, current_range in enumerate(scan.ranges):
            static_range = self.first_scan_data[i]

            if current_range < (static_range - self.moving_obstacle_threshold):
                bearing_i = scan.angle_min + i * scan.angle_increment
                moving_points.append(Point32(
                    x = current_range * cos(bearing_i),
                    y = current_range * sin(bearing_i),
                    z = 0.0
                ))

            else:
                bearing_i = scan.angle_min + i * scan.angle_increment
                stationary_points.append(Point32(
                    x = current_range * cos(bearing_i),
                    y = current_range * sin(bearing_i),
                    z = 0.0
                ))

        mov = PointCloud(header=scan.header, points=moving_points)
        stat = PointCloud(header=scan.header, points=stationary_points)

        return mov, stat
    
    def euclidean_clustering(self, moving_points: PointCloud, cluster_tolerance=0.51, min_size=8, max_size=300):
        ### Function defiend to make clusters for the identified people and calculate the cenbtroid for each clsuter formed
        points = [[p.x, p.y] for p in moving_points.points]    

        if not points:
            print("This was executed")
            return [], PointCloud(header=moving_points.header, points=[])

        points_array = np.array(points)
        unvisited = set(range(len(points_array)))
        clusters = []
        centroids = []

        for idx in range(len(points_array)):
            if idx not in unvisited:
                continue

            current_point = points_array[idx]
            neighbors = [idx]

            for other_idx in unvisited - {idx}:
                other_point = points_array[other_idx]
                if self.distance_2d(current_point, other_point) <= cluster_tolerance:
                    neighbors.append(other_idx)

            if min_size <= len(neighbors) <= max_size:
                clusters.append(neighbors)
                unvisited -= set(neighbors)
                
                # Compute centroid for the cluster
                cluster_points = points_array[neighbors]
                centroid = np.mean(cluster_points, axis=0)
                centroids.append(Point32(x = centroid[0],
                                         y = centroid[1],
                                         z = 0.0))
        
     
    # If there are centroids, pack them into a PointCloud message and return
    
            
        centroids_data = PointCloud(header=moving_points.header, points=centroids)

        return clusters, centroids_data

    
    def distance_2d(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def laser_scan_to_point_cloud(self, scan: LaserScan):
        points = []

        scan_length = len(scan.ranges)
        #print(type(scan.ranges))
        print(f"{scan.angle_min = }")
        #print(f"{scan.angle_max = }")

        print(f"{scan_length = }")

        for i in range(scan_length):
            range_i = scan.ranges[i]
            bearing_i = scan.angle_min+i*scan.angle_increment
            

            #print(f"{i = }")
            #print(f"{range_i = }")
            #print(f"{bearing_i = }")
            #print("#########")
            if not np.isfinite(scan.ranges[i]):
                continue
            points.append(Point32(
                x = range_i * cos(bearing_i),
                y = range_i * sin(bearing_i),
                z = 0.0
            ))
        return PointCloud(header=scan.header, points=points)

def main(args = None):
    rclpy.init(args = args)
    node = MovingObjectLocationDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()