using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;

public class JointStatePublisher : MonoBehaviour
{
    public string JointStateTopic = "/joint_states";
    public Transform[] Joints;  // Drag and drop the joints from the Unity editor

    private ROSConnection ros;
    private JointStateMsg jointStateMessage = new JointStateMsg();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(JointStateTopic);

        // Initialize the joint names based on the provided Transforms
        jointStateMessage.name = new string[Joints.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            jointStateMessage.name[i] = Joints[i].name;
        }
    }

    void Update()
    {
        PublishJointStates();
    }

    private void PublishJointStates()
    {
        // Manually setting the header timestamp
        DateTime currentTime = DateTime.Now;

        jointStateMessage.header = new HeaderMsg
        {
            stamp = new TimeMsg
            {
                sec = (uint)(currentTime - DateTime.Today).TotalSeconds,
                nanosec = (uint)((currentTime - DateTime.Today).TotalMilliseconds % 1000) * 1000000
            }

        };


        // Filling the positions
        double[] positions = new double[Joints.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            positions[i] = Joints[i].localEulerAngles.x * Mathf.Deg2Rad;  // Convert to radians
                                                                          
        }
        jointStateMessage.position = positions;

        // Publish the joint states
        ros.Publish(JointStateTopic, jointStateMessage);
    }
}
