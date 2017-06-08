import rospy
from object_location.srv import LocationQuery
from rail_object_detector.srv import SceneQuery


class ObjectLocationEngine():

    def __init__(self):
        self.object_detector_srv = rospy.get_param("~object_detector_service", "/detector_node/objects_in_scene")
        
        rospy.wait_for_service(self.object_detector_srv)
        self.object_detector = rospy.ServiceProxy(self.object_detector_srv, SceneQuery)

        self.mainQueryService = rospy.Service("location_query", LocationQuery, self.handleLocationQuery)

        rospy.spin()
    
    def handleLocationQuery(self, req):
        pass

    def askForBoundingBoxes(self):
        pass
