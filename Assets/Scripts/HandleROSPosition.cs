using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Std;




public class HandleROSPosition : MonoBehaviour
{
    ROSConnection ros;
    // Drag the virtual hand object here in the Unity editor
    public GameObject virtualHand;

    private string desired_ee_pose_topic = "ee_target_pose";
    private string joint_states_topic = "franka/joint_states";
    private List<UnityEngine.ArticulationBody> robot_joints;
    private bool firstUpdate = true;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        robot_joints = new List<UnityEngine.ArticulationBody>(GetComponentsInChildren<UnityEngine.ArticulationBody>());

        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>(joint_states_topic, HandleJointStateMessage);
    }

    void HandleJointStateMessage(JointStateMsg msg)
    {
        // Update the joint targets
        for (int i = 0; i < robot_joints.Count; i++)
        {
            var drive = robot_joints[i].xDrive;
            drive.target = (float)msg.position[i];
            robot_joints[i].xDrive = drive;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (firstUpdate)
        {
            for (int i = 0; i < robot_joints.Count; i++)
            {
                var drive = robot_joints[i].xDrive;
                drive.target = 0.0f;  // Default position for each joint
                robot_joints[i].xDrive = drive;
            }

            firstUpdate = false;
        }

        // Publish end-effector pose
        var poseMsg = new PoseMsg
        {
            position = new PointMsg 
            {
                x = virtualHand.transform.position.x,
                y = virtualHand.transform.position.y,
                z = virtualHand.transform.position.z
            },
            orientation = new QuaternionMsg 
            {
                x = virtualHand.transform.rotation.x,
                y = virtualHand.transform.rotation.y,
                z = virtualHand.transform.rotation.z,
                w = virtualHand.transform.rotation.w
            }
        };

        var poseStampedMsg = new PoseStampedMsg
        { 
            header = new HeaderMsg { frame_id = "base_link" },
            pose = poseMsg
        };

        // Publish current joint states
        var jointStateMsg = new JointStateMsg();
        jointStateMsg.position = new double[robot_joints.Count];
        for (int i = 0; i < robot_joints.Count; i++)
        {
            jointStateMsg.position[i] = robot_joints[i].xDrive.target;
        }
        
        ros.Publish(joint_states_topic, jointStateMsg);
        ros.Publish(desired_ee_pose_topic, poseStampedMsg);
    }
}
