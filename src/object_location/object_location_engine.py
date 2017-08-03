import numpy
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseStamped
from handle_detector.srv import HandleQuery
from object_location.msg import Object, Objects
from object_location.srv import LocationQuery, LocationQueryResponse
from rail_object_detector.srv import SceneQuery
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class ObjectLocationEngine():

    def __init__(self):
        self.object_detector_srv = rospy.get_param("~object_detector_service", "/detector_node/objects_in_scene")
        self.handle_detector_srv = rospy.get_param("~handle_detector_service", "/localization/handle_query");
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.publish_to_tf_tree = rospy.get_param("~publish_as_tf", True)
        
        if self.publish_to_tf_tree:
            rospy.loginfo("Will publish object locations to the TF tree as children of {}".format(self.global_frame))
        
        rospy.loginfo("Waiting for object detector service...")
        rospy.wait_for_service(self.object_detector_srv)
        rospy.loginfo("Object detector service loaded")

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf.TransformBroadcaster()

        # Incoming messages and services
        self.objects = []
        self.pointClouds = []
        self.object_detector = rospy.ServiceProxy(self.object_detector_srv, SceneQuery)
        self.handle_detector = rospy.ServiceProxy(self.handle_detector_srv, HandleQuery)
        
        self.frame = None
        rospy.Subscriber("/kinect/hd/points", PointCloud2, self.pointCloudUpdate, queue_size=1)

        # Outgoing messages and services
        self.main_query_service = rospy.Service("location_query", LocationQuery, self.handleLocationQuery)
        self.object_publisher = rospy.Publisher("object_location", Objects, queue_size=1)

        if self.publish_to_tf_tree:
            self.tfBroadcaster = tf.TransformBroadcaster()
            rate = rospy.Rate(5)
            while not rospy.is_shutdown():
                for item in self.objects:
                    position = (item.pose.position.x,
                                item.pose.position.y,
                                item.pose.position.z)
                    rotation = (item.pose.orientation.x,
                                item.pose.orientation.y,
                                item.pose.orientation.z,
                                item.pose.orientation.w,)

                    self.tfBroadcaster.sendTransform(position, rotation, rospy.Time.now(), item.label, self.global_frame)

                self.object_publisher.publish(Objects(objects=self.objects))
                
                for pointCloud in self.pointClouds:
                    # This is published mostly as a way to debug the segmented point clouds in RViz
                    rospy.Publisher("object_location_cloud/{}".format(pointCloud["label"]), PointCloud2, queue_size=1).publish(pointCloud["pointCloud"])
                rate.sleep()
        else:
            rospy.spin()
    
    def handleLocationQuery(self, req):
        self.objects = []

        rospy.loginfo("Sending query to rail_object_detector...")
        result = self.object_detector()
        rospy.loginfo("Query result received of length {}".format(len(result.objects)))

        rospy.loginfo("Sending query to handle_detector...")
        handleResult = self.handle_detector()
        rospy.loginfo("Query result received of length {}".format(len(handleResult.handles)))
        mergedHandles = []
        
        if self.frame is None:
            rospy.logwarn("No depth frame processed yet")
            return

        for item in result.objects:
            positionTuple = self.frame[item.centroid_y][item.centroid_x]
            rotationTuple = (0, 0, 0, 1)

            croppedPoints = []
            for y in xrange(item.right_top_y, item.left_bot_y):
                for x in xrange(item.left_bot_x, item.right_top_x):
                    croppedPoints.append(self.frame[y][x])
            
            croppedPointCloud = pc2.create_cloud_xyz32(Header(frame_id="kinect_rgb_optical_frame"), croppedPoints)

            self.pointClouds.append({
                "label": item.label,
                "probability": item.probability,
                "pointCloud": croppedPointCloud
            })

            remappedObject = Object()
            existingLabels = sum(1 for x in self.objects if x.label == item.label)
            if existingLabels == 0:
                remappedObject.label = item.label
            else:
                remappedObject.label = "{}_{}".format(item.label, existingLabels)
            remappedObject.probability = item.probability
            remappedObject.cloud = croppedPointCloud

            (remappedObject.pose.position.x,
             remappedObject.pose.position.y,
             remappedObject.pose.position.z) = positionTuple
            rotationTuple = (0, 0, 0, 1) # No rotation for now

            print("Found \"{}\" with probability {} and centroid (x: {}, y: {})".format(item.label, item.probability, item.centroid_x, item.centroid_y))
            print("Full 3D position for this object is:\n{}".format(remappedObject.pose.position))

            # Find absolute position for this object (in map frame)
            now = rospy.Time.now()
            self.tfBroadcaster.sendTransform(positionTuple, rotationTuple, now, item.label, "kinect_rgb_optical_frame")
            absoluteTransform = self.tfBuffer.lookup_transform(self.global_frame, item.label, rospy.Time(0), timeout=rospy.Duration(5)).transform

            (remappedObject.pose.position.x,
             remappedObject.pose.position.y,
             remappedObject.pose.position.z) = (absoluteTransform.translation.x, absoluteTransform.translation.y, absoluteTransform.translation.z)
             
            (remappedObject.pose.orientation.x,
             remappedObject.pose.orientation.y,
             remappedObject.pose.orientation.z,
             remappedObject.pose.orientation.w) = rotationTuple
            
            MATCHING_DISTANCE = 0.1
            for handleItem in handleResult.handles:
                if handleItem in mergedHandles:
                    continue
                
                xThresholdSatisfied = abs(remappedObject.pose.position.x - handleItem.pose.position.x) < MATCHING_DISTANCE
                yThresholdSatisfied = abs(remappedObject.pose.position.y - handleItem.pose.position.y) < MATCHING_DISTANCE
                zThresholdSatisfied = abs(remappedObject.pose.position.z - handleItem.pose.position.z) < MATCHING_DISTANCE

                if xThresholdSatisfied and yThresholdSatisfied and zThresholdSatisfied:
                    remappedObject.pose.orientation = handleItem.pose.orientation;
                    print("Found matching handle, setting rotation to (x: {}, y: {}, z: {}, w: {})".format(
                        remappedObject.pose.orientation.x,
                        remappedObject.pose.orientation.y,
                        remappedObject.pose.orientation.z,
                        remappedObject.pose.orientation.w
                    ))
                    mergedHandles.append(handleItem)

            self.objects.append(remappedObject)
        # Add handles without corresponding detector results as objects
        for i, handleItem in enumerate(handleResult.handles):
            if handleItem in mergedHandles:
                continue

            remappedObject = Object()
            remappedObject.label = "handle_object_{}".format(i)
            remappedObject.probability = 0
            remappedObject.cloud = PointCloud2()
            remappedObject.pose.position = handleItem.pose.position
            remappedObject.pose.orientation = handleItem.pose.orientation
            self.objects.append(remappedObject)

        self.object_publisher.publish(Objects(objects=self.objects))

        result = LocationQueryResponse()
        result.header = Header()
        result.objects = self.objects
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
