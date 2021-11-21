import rospy
import tf
from tf import transformations as t
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovariance, TransformStamped, Vector3, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
import time

global tf_pub
global tf_listener

global head_yaw_pose

def tf_callback(msg):
    global tf_pub
    global tf_listener
    global head_yaw_pose

    quatenrnion = [-0.684, 0.721, 0.102, 0.036]
    if msg.transforms[0].child_frame_id == "vr_tracker_right":
        tf_msg = TFMessage()
        msg_stamp = TransformStamped()
        #msg_stamp.transform.translation = msg.transforms[0].transform.translation
        tra, rot = tf_listener.lookupTransform("/map", "/vr_tracker_right", rospy.Time(0))
        msg_stamp.transform.translation.x = tra[0]
        msg_stamp.transform.translation.y = tra[1]
        msg_stamp.transform.translation.z = 0

        #rot = quaternioin_to_list(msg.transforms[0].transform.rotation)
        head_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, head_yaw_pose, axes="sxyz"))
        correct_head = t.concatenate_matrices(t.quaternion_matrix(rot), t.quaternion_matrix(head_quanternion))
        new_q = t.concatenate_matrices(correct_head, t.quaternion_matrix(quatenrnion))
        inv = t.quaternion_from_matrix(new_q)
        msg_stamp.transform.rotation.x = inv[0]
        msg_stamp.transform.rotation.y = inv[1]
        msg_stamp.transform.rotation.z = inv[2]
        msg_stamp.transform.rotation.w = inv[3]

        msg_stamp.child_frame_id = 'base'
        msg_stamp.header.frame_id = 'map'
        msg_stamp.header.stamp = rospy.Time.now()
        tf_msg.transforms.append(msg_stamp)
        print(tf_msg)
        tf_pub.publish(tf_msg)

def quaternioin_to_list(q):
    return [q.x, q.y, q.z, q.w]

def joint_state_callback(msg):
    global head_yaw_pose
    states = dict(zip(msg.name, msg.position))
    head_yaw_pose = states['HeadYaw']

if __name__ == '__main__':
    rospy.init_node('tf_base')
    tf_listener = tf.TransformListener()
    time.sleep(1)
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    rospy.spin()
