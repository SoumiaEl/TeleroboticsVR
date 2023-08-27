using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class HandlROSPositionInter : MonoBehaviour
{
    ROSConnection ros;
    // Drag the virtual hand object here in the Unity editor
    [SerializeField]
    GameObject virtualHand;

    [SerializeField]
    GameObject base_link;

    [SerializeField]
    GameObject endEffector;

    // Articulation Bodies
    List<ArticulationBody> m_JointArticulationBodies;

    public static readonly string[] LinkNames =
        { "world/panda_link0/panda_link1", "world/panda_link0/panda_link1/panda_link2", "world/panda_link0/panda_link1/panda_link2/panda_link3", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7" };

    public static readonly string[] RosLinkNames =
    { "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7" };


    private string robotNamespace = "franka";
    private string desired_ee_pose_topic;
    private string desired_joint_states_topic;
    private UrdfJointRevolute[] robot_joints;
    private int nb_robots_joints = 7;
    private float distanceThreshold = 0.05f;
    // Add this at the class level
    private JointStateMsg lastJointStateMsg = null;

    private float[] targetJointPositions;
    private float interpolationRate = 0.05f;  // Ajustez cette valeur pour changer la vitesse d'interpolation





    // Start is called before the first frame update
    void Start()
    {

        ros = ROSConnection.GetOrCreateInstance();

        desired_ee_pose_topic = "ee_target_pose";
        desired_joint_states_topic = robotNamespace + "/desired_joint_state";

        // Get UrdfJointRevolute components instead of ArticulationBody components
        robot_joints = new UrdfJointRevolute[nb_robots_joints];



        m_JointArticulationBodies = new List<ArticulationBody>(GetComponentsInChildren<ArticulationBody>());

        for (var i = 0; i < nb_robots_joints; i++)
        {
            Debug.Log("Test iteration : " + i);
            // Find the joint by its name and add it to the array at the corresponding index
            m_JointArticulationBodies[i] = this.gameObject.transform.Find(LinkNames[i]).GetComponent<ArticulationBody>();
            robot_joints[i] = this.gameObject.transform.Find(LinkNames[i]).GetComponent<UrdfJointRevolute>();
        }

        ros.RegisterPublisher<JointStateMsg>(robotNamespace + "/current_joint_states");
        ros.RegisterPublisher<PoseStampedMsg>("ee_target_pose");
        ros.RegisterPublisher<TwistMsg>("ee_target_velocity");


        // Initialize joint positions
        float[] initialJointPositions = { 0.0f, -0.7f, 0.0f, -1.8f, 0.0f, 1.1f, 0.0f };

        // Ensure that the array of initial joint positions is the correct size
        if (initialJointPositions.Length != nb_robots_joints)
        {
            Debug.LogError("Array of initial joint positions has incorrect size");
            return;
        }

        // Update the target positions for each joint
        for (int i = 0; i < nb_robots_joints; i++)
        {
            var drive = m_JointArticulationBodies[i].xDrive;
            drive.target = initialJointPositions[i] * Mathf.Rad2Deg; // Convert to degrees as Unity uses degrees for rotation
            m_JointArticulationBodies[i].xDrive = drive; // Set the target rotation for the joint
            Debug.Log($"For joint {i}, xDrive.target is set to: {m_JointArticulationBodies[i].xDrive.target}");

        }

        foreach (UrdfJointRevolute joint in robot_joints)
        {
            if (joint != null)
            {
                Debug.Log("Joint name: " + joint.name);
                Debug.Log("Joint position: " + joint.transform.position);
                Debug.Log("Joint rotation: " + joint.transform.rotation);
                // ... and so on for other properties you're interested in
            }
            else
            {
                Debug.Log("Joint is null!");
            }
        }

        targetJointPositions = new float[nb_robots_joints];
        for (int i = 0; i < nb_robots_joints; i++)
        {
            targetJointPositions[i] = m_JointArticulationBodies[i].xDrive.target;
            Debug.Log($"For joint {i}, jointPosition[0] is: {targetJointPositions[i]}");
        }

        Debug.Log($"targetJointPositions initialized with length: {targetJointPositions.Length}");

        // Invoke these methods after a delay of 0.5 second.
        Invoke("CurrentJointState", 0.5f);
        Debug.Log("Je trouve les informations de mes joints current ? ");
        Invoke("SendTargetPose", 0.5f);

        ros.Subscribe<JointStateMsg>(desired_joint_states_topic, HandleDesiredJointStateMessage);

    }

    void CurrentJointState()
    {
        // Publish initial joint states

        var jointStateMsg = new JointStateMsg
        {
            position = new double[nb_robots_joints],
            name = new string[nb_robots_joints],
            velocity = new double[nb_robots_joints],
            effort = new double[nb_robots_joints]
        };

        jointStateMsg.header.frame_id = "panda_link0";

        for (int i = 0; i < nb_robots_joints; i++)
        {

            jointStateMsg.name[i] = RosLinkNames[i];

            // Get velocity and position from UrdfJointRevolute component
            jointStateMsg.velocity[i] = robot_joints[i].GetVelocity();
            jointStateMsg.position[i] = robot_joints[i].GetPosition();

            Debug.Log("Position du Joint" + i + ":" + jointStateMsg.position[i]);
        }

        Debug.Log(jointStateMsg);
        ros.Publish(robotNamespace + "/current_joint_states", jointStateMsg);

        Debug.Log("Test Position de l'effecteur :" + endEffector.transform.position);
        Debug.Log("Test Rotation de l'effecteur :" + endEffector.transform.rotation);
    }


    void SendTargetPose()
    {


        // Créer un PoseMsg avec la position et l'orientation ROS
        var poseMsg = new PoseMsg
        {
            position = virtualHand.transform.position.To<FLU>(),

            orientation = virtualHand.transform.rotation.To<FLU>()
        };

        // Créer un PoseStampedMsg et le publier
        var poseStampedMsg = new PoseStampedMsg
        {
            header = new HeaderMsg { frame_id = "panda_link0" },
            pose = poseMsg
        };

        Debug.Log("Position souhaitée : " + poseStampedMsg);

        ros.Publish(desired_ee_pose_topic, poseStampedMsg);

        Debug.Log("La target est publiée ?");
    }


    void HandleDesiredJointStateMessage(JointStateMsg msg)
    {

        // Update the last joint state message
        lastJointStateMsg = msg;

        Debug.Log("Voici le message reçu sous ROS:" + msg);

        // Check that the message itself is not null
        if (msg == null)
        {
            Debug.Log("Received null JointStateMsg");
            return;
        }

        // Vérifiez que le message a le bon nombre de positions
        if (msg.position.Length != nb_robots_joints)
        {
            Debug.Log("Received JointStateMsg with incorrect number of positions");
            return;
        }


        // Set the joint values for every joint
        for (var joint = 0; joint < RosLinkNames.Length; joint++)
        {
            targetJointPositions[joint] = (float)msg.position[joint] * Mathf.Rad2Deg;
        }

    }


    // Update is called once per frame
    void Update()
    {
        // Get the position of the virtual hand and the robot
        Vector3 handPosition = virtualHand.transform.position;
        Vector3 robotPosition = endEffector.transform.position;

        // Calculate the distance between the hand and the robot
        float distance = Vector3.Distance(handPosition, robotPosition);

        // If the distance is greater than the threshold, update the robot joint positions
        if (distance > distanceThreshold)
        {
            Invoke("CurrentJointState", 0.0f);
            Invoke("SendTargetPose", 0.5f);

            // Handle the most recent joint state message
            HandleDesiredJointStateMessage(lastJointStateMsg);
        }

        // Interpolate joint positions
        for (int i = 0; i < nb_robots_joints; i++)
        {
            var currentJointPosition = m_JointArticulationBodies[i].jointPosition[0];
            var newJointPosition = Mathf.Lerp(currentJointPosition, targetJointPositions[i], interpolationRate);

            var jointDrive = m_JointArticulationBodies[i].xDrive;
            jointDrive.target = newJointPosition;
            m_JointArticulationBodies[i].xDrive = jointDrive;
        }
    }

}
