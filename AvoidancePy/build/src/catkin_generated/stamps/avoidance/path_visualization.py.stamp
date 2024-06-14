import rospy
import numpy as np
import matplotlib.pyplot as plt 
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class CoordinateConverter:
    def __init__(self, origin_lat: float = 0, origin_lon: float = 0, orientation_deg: float = 0):
        self.set_origin(origin_lat, origin_lon, orientation_deg)
        
    def set_origin(self, origin_lat: float = 0, origin_lon: float = 0, orientation_deg: float = 0):
        self.origin_lat = origin_lat 
        self.origin_lon = origin_lon 
        self.orientation_rad = np.radians(orientation_deg)

    def to_cartesian(self, lat: float, lon: float):
        R = 6378137.0  # Radius of the Earth in meters
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        origin_lat_rad = np.radians(self.origin_lat)
        origin_lon_rad = np.radians(self.origin_lon)

        x = R * (lon_rad - origin_lon_rad) * np.cos(origin_lat_rad)
        y = R * (lat_rad - origin_lat_rad)
        
        return x, y

    def to_local_frame(self, lat, lon):
        x, y = self.to_cartesian(lat, lon)

        rotation_matrix = np.array([
            [np.cos(self.orientation_rad), -np.sin(self.orientation_rad)],
            [np.sin(self.orientation_rad), np.cos(self.orientation_rad)]
        ])

        local_coordinates = np.dot(rotation_matrix, [x, y])
        
        return local_coordinates[0], local_coordinates[1]

class PathVisualizer:
    def __init__(self) -> None:
        self._original_wp_pub = rospy.Publisher('/original_wp_vis', Marker, queue_size=1)
        self._adopted_wp_pub = rospy.Publisher('/adopted_wp_vis', Marker, queue_size=1)
        self._mission_wp_pub = rospy.Publisher('/mission_waypoints_vis', MarkerArray, queue_size=1)
        
        self._actual_path_pub = rospy.Publisher('/actual_path_vis', Marker, queue_size=1)
        self._wp_path_pub = rospy.Publisher('/wp_path_vis', Marker, queue_size=1)
        self._adopted_path_pub = rospy.Publisher('/adopted_path_vis', Marker, queue_size=1)
        self._path_pub = rospy.Publisher('/path_vis', MarkerArray, queue_size=1)
        self.path_length_actual_ = 0
        self._path_length_wp = 0
        self._path_length_adopted = 0
        self._converter = CoordinateConverter()
        
        # init plot figure for vfh viualization
        self.fig = plt.figure()
        self.ax = self.fig.subplots(nrows=2, ncols=2)
        self.fig.delaxes(self.ax[1][0])
        self.fig.delaxes(self.ax[0][0])
        self.ax[0][0] = self.fig.add_subplot(2, 2, 1, projection='polar')
        self.ax[1][1].grid()
        self.ax[0][1].grid()
    
    def publish_mission_waypoints(self, waypoints, origin):
        self._converter.set_origin(origin_lat=origin.latitude,
                                   origin_lon=origin.longitude,
                                   orientation_deg=0)
        mission_points = MarkerArray()
        for id, point in enumerate(waypoints.waypoints):
            m = Marker()
            m.header.frame_id = "map"
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 0.5
            self._set_mission_marker_color(m, id, waypoints.current_seq)
            x, y = self._converter.to_cartesian(point.x_lat, point.y_long)
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = point.z_alt
            m.id = id
            mission_points.markers.append(m)
            
        self._mission_wp_pub.publish(mission_points)
        
    def _set_mission_marker_color(self, marker, id: int, cur_point_id: int):
        if cur_point_id == id:
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        elif cur_point_id > id:
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        
    def update_actual_path(self, prev_pose, current_pose):
        # publish actual path
        path_actual_marker = Marker()
        path_actual_marker.header.frame_id = "map"
        path_actual_marker.header.stamp = rospy.Time().now()
        path_actual_marker.id = self.path_length_actual_
        path_actual_marker.type = Marker.LINE_STRIP
        path_actual_marker.action = Marker.ADD
        path_actual_marker.pose.orientation.w = 1.0
        path_actual_marker.scale.x = 0.03
        path_actual_marker.color.a = 1.0
        path_actual_marker.color.r = 0.0
        path_actual_marker.color.g = 1.0
        path_actual_marker.color.b = 0.0
        path_actual_marker.points.append(prev_pose)
        path_actual_marker.points.append(current_pose)
        self._actual_path_pub.publish(path_actual_marker)
        self.path_length_actual_ += 1
        
    def update_waypoint_path(self, prev_wp, current_wp):
        # publish wp path
        wp_path_marker = Marker()
        wp_path_marker.header.frame_id = "map"
        wp_path_marker.header.stamp = rospy.Time().now()
        wp_path_marker.id = 0
        wp_path_marker.type = Marker.LINE_STRIP
        wp_path_marker.action = Marker.ADD
        wp_path_marker.pose.orientation.w = 1.0
        wp_path_marker.scale.x = 0.03
        wp_path_marker.color.a = 1.0
        wp_path_marker.color.r = 1.0
        wp_path_marker.color.g = 0.0
        wp_path_marker.color.b = 0.0
        wp_path_marker.points.append(prev_wp)
        wp_path_marker.points.append(current_wp)
        self._wp_path_pub.publish(wp_path_marker)
        self._path_length_wp += 1
        
    def update_adopted_path(self, current_pose, current_target):
        # publish target path
        target_path_marker = Marker()
        target_path_marker.header.frame_id = "map"
        target_path_marker.header.stamp = rospy.Time().now()
        target_path_marker.id = 0
        target_path_marker.type = Marker.LINE_STRIP
        target_path_marker.action = Marker.ADD
        target_path_marker.pose.orientation.w = 1.0
        target_path_marker.scale.x = 0.03
        target_path_marker.color.a = 1.0
        target_path_marker.color.r = 0.0
        target_path_marker.color.g = 0.0
        target_path_marker.color.b = 1.0
        target_path_marker.points.append(current_pose)
        target_path_marker.points.append(current_target)
        self._adopted_path_pub.publish(target_path_marker)
        self._path_length_adopted += 1

    def draw_path(self, path: list) -> None:
        path_markers = MarkerArray()
        for index, item in enumerate(path):
            if index == len(path)-1: break
            m = Marker()
            m.header.frame_id = "map"
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.id = index
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            cur_p = Point(path[index][0], path[index][1], path[index][2])
            next_p = Point(path[index+1][0], path[index+1][1], path[index+1][2])
            m.points.append(cur_p)
            m.points.append(next_p)
            path_markers.markers.append(m)
          
        m = Marker()
        m.scale.x = 0.1
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.header.frame_id = "map"
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.id = len(path)+1
        m.pose.orientation.w = 1.0
        m.pose.position.x = path[0][0]
        m.pose.position.y = path[0][1]
        m.pose.position.z = path[0][2]
        path_markers.markers.append(m)
        m = Marker()
        m.scale.x = 0.1
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.header.frame_id = "map"
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.id = len(path)+2
        m.pose.position.x = path[-1][0]
        m.pose.position.y = path[-1][1]
        m.pose.position.z = path[-1][2]
        path_markers.markers.append(m) 
        self._path_pub.publish(path_markers)