import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Pose, Quaternion
from object_location.msg import Object
from object_location.srv import LocationQuery, LocationQueryResponse
from rail_object_detector.srv import SceneQuery
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class ObjectLocationEngine():

    def __init__(self):
        self.object_detector_srv = rospy.get_param("~object_detector_service", "/detector_node/objects_in_scene")
        
        rospy.loginfo("Waiting for object detector service...")
        rospy.wait_for_service(self.object_detector_srv)
        rospy.loginfo("Object detector service loaded")

        # Incoming messages and services
        self.object_detector = rospy.ServiceProxy(self.object_detector_srv, SceneQuery)
        
        self.frame = None
        rospy.Subscriber("/kinect/hd/points", PointCloud2, self.pointCloudUpdate, queue_size=1)

        # Outgoing messages and services
        self.mainQueryService = rospy.Service("location_query", LocationQuery, self.handleLocationQuery)
        
        # For testing only
        #self.handleLocationQuery(None)

        rospy.spin()
    
    def handleLocationQuery(self, req):
        rospy.loginfo("Sending query...")
        result = self.object_detector()
        rospy.loginfo("Query result received of length {}".format(len(result.objects)))
        
        if self.frame is None:
            rospy.logwarn("No depth frame processed yet")
            return

        objects = []
        for item in result.objects:
            remappedObject = Object()
            remappedObject.label = item.label
            remappedObject.probability = item.probability

            (remappedObject.pose.position.x,
             remappedObject.pose.position.y,
             remappedObject.pose.position.z) = self.frame[item.centroid_y][item.centroid_x]

            print("Found \"{}\" with probability {} and centroid (x: {}, y: {})".format(item.label, item.probability, item.centroid_x, item.centroid_y))
            print("Full 3D position for this object is:\n{}".format(remappedObject.pose.position))

            objects.append(remappedObject)

        result = LocationQueryResponse()
        result.header = Header()
        result.objects = objects
        return result

    def pointCloudUpdate(self, data):
        points =  pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        frame = []
        row = []
        
        for point in points:
            if len(row) >= data.width:
                frame.append(row)
                row = []
            row.append(point)
        frame.append(row)

        self.frame = frame
