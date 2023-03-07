#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
# Read data

# Define ECE parameters
min_cluster_size = 5
max_cluster_size = 500
cluster_distance = 0.1

# Initialize list of clusters
clusters = []

# Process each scan data packet
def cluster_processing(data):
    # Convert scan data to numpy array of (x, y, z) coordinates
    points = np.array([[np.sin(np.deg2rad(d[1]))*d[2], np.cos(np.deg2rad(d[1]))*d[2], 0] for d in data.intensities])
    
    # Initialize list of cluster indices
    cluster_indices = []
    
    # Perform ECE algorithm
    for i, point in enumerate(points):
        # Find all neighboring points within cluster distance
        neighbors = np.where(np.sum((points - point)**2, axis=1) < cluster_distance**2)[0]
        
        # If there are enough neighboring points, add them to cluster
        if len(neighbors) >= min_cluster_size:
            # Exclude points that already belong to another cluster
            neighbors = [n for n in neighbors if n not in cluster_indices]
            
            # If the cluster is not too big, add it to list of clusters
            if len(neighbors) <= max_cluster_size:
                clusters.append(points[neighbors])
                cluster_indices.extend(neighbors)
    
    # Do something with the clusters (e.g. publish to ROS)
    pub = rospy.Publisher('clusters', float, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    pub.publish(clusters)
    rospy.loginfo(clusters)
    
# Stop scanning and disconnect from the RPLidar sensor

def main():
    rospy.init_node('cluster_extraction_test', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, cluster_processing)

    rospy.loginfo("Starting cluster_extraction_test.py")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass