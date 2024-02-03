import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from example_interfaces.msg import Int64
import numpy as np



class Track:
    def __init__(self, initial_position):
        self.position = initial_position
        self.velocity = np.array([0.0, 0.0])  # Assuming 2D x, y movement
        self.updated = True
        self.lifetime = 20 # Number of frames a track is retained without an update

    def predict(self):
        """Predicts the next position using constant velocity model."""
        return self.position + self.velocity

    def update(self, new_position):
        """Updates the position and velocity of the track."""
        new_position = np.array(new_position)
        self.velocity = new_position - self.position
        self.position = new_position  



class TrackMovingObject(Node):
    def __init__(self):
        super().__init__('detect_moving_objects')
        self.subscription = self.create_subscription(PointCloud, '/person_locations', self.TrackerCallback, 10)
        self.publisher_count = self.create_publisher(Int64, '/person_count', 10)
        self.first_scan_data = None
        self.moving_obstacle_threshold = 0.3
        self.header = None
        self.tracks = []
        self.unique_people_count = 0
    

    def TrackerCallback(self,centroids):

        current_centroids = [[p.x, p.y] for p in centroids.points]
        # Predict next positions of existing tracks
        # 1. Predict the next position for all tracks
        predicted_position = [track.predict() for track in self.tracks]

        # 2. Associate current centroids with predicted positions of existing tracks
        used_centroids = []
        for i, predicted_pos in enumerate(predicted_position):
            min_dist = float('inf')
            best_match = None
            for j, centroid in enumerate(current_centroids):
                dist = np.linalg.norm(predicted_pos - centroid)
                if dist < min_dist and j not in used_centroids:
                    min_dist = dist
                    best_match = j

            # Update the track if a close centroid is found
            if best_match is not None and min_dist < 0.9:  # Distance threshold can be adjusted
                self.tracks[i].update(np.array(current_centroids[best_match]))
                used_centroids.append(best_match)
    
        # 3. Create new tracks for unmatched centroids
        for i, centroid in enumerate(current_centroids):
            if i not in used_centroids:
                self.tracks.append(Track(centroid))
                self.unique_people_count += 1

        # Decrement lifetime of tracks that haven't been updated
        for track in self.tracks:
            if not track.updated:
                track.lifetime -= 1

        # Remove tracks with lifetime <= 0
        self.tracks = [track for track in self.tracks if track.lifetime > 0]


        self.get_logger().info(f"Total unique people counted: {self.unique_people_count}")
        count_msg = Int64()
        count_msg.data = self.unique_people_count
        self.publisher_count.publish(count_msg)


def main(args = None):
    rclpy.init(args = args)
    node = TrackMovingObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

