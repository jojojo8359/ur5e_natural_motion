#! /usr/bin/env python

import rospy
import sensor_msgs.msg
import gazebo_msgs.msg
import tf2_msgs.msg
import geometry_msgs.msg
import tf2_ros

def main():
	rospy.init_node('box_publisher_node', anonymous=True)

	pub_tf = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)
	gazebo_model_state_pub = rospy.Publisher('/gazebo/set_model_state', gazebo_msgs.msg.ModelState, queue_size=1)

	tfBuffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tfBuffer)

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		t = geometry_msgs.msg.TransformStamped()
		t.header.frame_id = "wrist_3_link"
		t.header.stamp = rospy.Time.now()
		t.child_frame_id = "demo_cube"
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.1

		t.transform.rotation.x = 0.0
		t.transform.rotation.y = 0.0
		t.transform.rotation.z = 0.0
		t.transform.rotation.w = 1.0

		tfm = tf2_msgs.msg.TFMessage([t])
		pub_tf.publish(tfm)

		try:
			trans = tfBuffer.lookup_transform("demo_cube", "base_link", rospy.Time())  # type: geometry_msgs.msg.TransformStamped
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		
		model_state = gazebo_msgs.msg.ModelState()
		model_state.model_name = "demo_cube"

		cube_pose = geometry_msgs.msg.Pose()
		cube_pose.position.x = trans.transform.translation.x
		cube_pose.position.y = trans.transform.translation.y
		cube_pose.position.z = trans.transform.translation.z
		cube_pose.orientation = trans.transform.rotation

		model_state.pose = cube_pose
		model_state.reference_frame = "robot::base_link"

		gazebo_model_state_pub.publish(model_state)

		rate.sleep()

if __name__ == '__main__':
	main()
