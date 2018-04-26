
import rospy
import yaml
import tf

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def setattrs(_self, **kwargs):
    for k,v in kwargs.items():
        setattr(_self, k, v)


def openYaml(yaml_path):
    #open yaml file
    print "Markers yaml path is {}".format(yaml_path)
    with open(yaml_path, 'r') as stream:
        # check first line to ensure format
        if (stream.readline() != "# VITAG POSITION YAML\n"):
            print " Wrong format! Wrong Vitag position yaml file was selected!"
            exit(0)
        try:
            yaml_obj = yaml.load(stream)
            print "yaml open successfully!"
        except yaml.YAMLError as exc:
            print " Error in reading yaml file!"
            print " Check the format and syntax of yaml"
            exit(0)
        
    return yaml_obj



def create_VITagMarkers(yaml_obj, delta):
    # create a grey box marker
    box_marker = Marker()
    box_marker.header.frame_id = "world"
    box_marker.ns = "vitag_marker"
    box_marker.type = Marker.CUBE_LIST

    setattrs( box_marker.scale , x=0.25, y=0.02, z=0.1)
    setattrs( box_marker.color , r=0.0, g=1, b=0.2, a=1.0)
    setattrs( box_marker.pose.position , x=0.1, y=0, z=0)
    setattrs( box_marker.pose.orientation , x=0, y=0, z=0, w=1) #change orientation of yaw in z-axis
    print "VITag Markers is starting"
    
    #creare lists of points for VITags
    print "Inserting total {} markers into scene...".format(len(yaml_obj['markers']))
    for idx in yaml_obj["markers"]:
        info = yaml_obj["markers"][idx]
        _x = info['x']/100.0 #convert to meters
        _y = info['y']/100.0 + delta
        
        marker_point = Point()
        setattrs( marker_point , x=_x, y= _y, z=0)
        box_marker.points.append (marker_point)
        
        print idx, ":\t", info['x'], info['y'], info['yaw']

    return box_marker
    


def config_NavigPath():
    # create a dot for line
    dot = Marker()
    dot.header.frame_id = "world"
    dot.ns = "vitag_marker"
    dot.type = Marker.LINE_STRIP
    setattrs( dot.scale , x=0.02, y=0.02, z=0.01)
    setattrs( dot.color , r=0.5, g=0.5, b=0, a=1.0)
    setattrs( dot.pose.position , x=0, y=0, z=0)
    return dot


def add_NagPath(NagPath, _x, _y):
    dot_point = Point()
    setattrs( dot_point , x=_x, y= _y, z=0)
    if len(NagPath.points) >= maxNagPoints: # remove first element so not too overwhelmed!
        del NagPath.points[0]
    NagPath.points.append (dot_point)
    return NagPath


if __name__=="__main__":

    rospy.init_node("rviz_markers")
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=20)  
    track_publisher = rospy.Publisher('tracking_line', Marker, queue_size=20)  
    tf_listener = tf.TransformListener()

    yaml_path = "../tag_detection/markers.yaml"

    #start process
    yaml_obj = openYaml(yaml_path)
    VITagMarkers = create_VITagMarkers(yaml_obj, 0)
    camera_NagPath = config_NavigPath()
    maxNagPoints = 100

    print "here we go!!"
    print VITagMarkers.points

    while 1:
        try:
            (trans,rot) = tf_listener.lookupTransform("/world", '/base_link', rospy.Time(0))
            print "tf is running: {} {}".format(trans[0], trans[1])
            camera_NagPath = add_NagPath(camera_NagPath, trans[0], trans[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        marker_publisher.publish(VITagMarkers)
        track_publisher.publish(camera_NagPath)

        # break
        # VITagMarkers.pose.position.x = VITagMarkers.pose.position.x -0.05
        rospy.sleep(0.5)

    rospy.spin()

