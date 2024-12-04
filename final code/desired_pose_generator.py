import rospy
from std_msgs.msg import String
import numpy as np
import re

class DesiredPoseGenerator:
    def __init__(self):
        rospy.init_node('desired_pose_generator', anonymous=True)
        self.pbvs_subscriber = rospy.Subscriber('/pbvs_data', String, self.pbvs_data_callback)
        self.target_pose = None
        self.target_marker_id = '123'  # Update with your target marker ID

    def pbvs_data_callback(self, msg):
        try:
            data = msg.data
            marker_id = int(re.search(r"Marker ID: (\d+)", data).group(1))
            if str(marker_id) == self.target_marker_id:
                x = float(re.search(r"x:\s*(-?\d+\.?\d*)", data).group(1))
                y = float(re.search(r"y:\s*(-?\d+\.?\d*)", data).group(1))
                z = float(re.search(r"z:\s*(-?\d+\.?\d*)", data).group(1))
                roll = float(re.search(r"Roll:\s*(-?\d+\.?\d*)", data).group(1))
                pitch = float(re.search(r"Pitch:\s*(-?\d+\.?\d*)", data).group(1))
                yaw = float(re.search(r"Yaw:\s*(-?\d+\.?\d*)", data).group(1))

                # Save the pose components
                self.target_pose = {
                    "tvec": np.array([x, y, z]),  # Translation vector
                    "euler_angles": np.array([roll, pitch, yaw]),  # Euler angles (roll, pitch, yaw)
                }
                rospy.loginfo(f"Target pose captured: {self.target_pose}")
        except AttributeError:
            rospy.logerr("Error parsing pbvs_data message. Check the format.")

    def save_desired_pose(self):
        if self.target_pose:
            try:
                # Save the desired pose components
                np.save('bin/desired_realworld_tvec.npy', self.target_pose['tvec'])
                np.save('bin/desired_euler_angles.npy', self.target_pose['euler_angles'])

                # Combine tvec and euler_angles into a single pose file for flexibility
                pose_combined = np.hstack([self.target_pose['tvec'], self.target_pose['euler_angles']])
                np.save('bin/desired_pose.npy', pose_combined)

                rospy.loginfo("Desired pose files saved successfully.")
            except Exception as e:
                rospy.logerr(f"Error saving desired pose files: {e}")
        else:
            rospy.logwarn("No target pose data available to save.")

def main():
    generator = DesiredPoseGenerator()
    rospy.loginfo("Desired Pose Generator is running...")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Saving desired pose files...")
        generator.save_desired_pose()
        rospy.loginfo("Node terminated.")

if __name__ == '__main__':
    main()
