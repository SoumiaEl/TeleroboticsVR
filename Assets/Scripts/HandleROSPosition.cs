using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Std;




public class HandleROSPosition : MonoBehaviour
{
    ROSConnection ros;
    // Drag the virtual hand object here in the Unity editor
    [SerializeField]
    GameObject virtualHand;

    [SerializeField]
    GameObject base_link;

    [SerializeField]
    GameObject endEffector;

    [SerializeField]
    GameObject panda;

    // Articulation Bodies
    List <ArticulationBody> m_JointArticulationBodies;

    public static readonly string[] LinkNames =
        { "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", "/panda_link5", "/panda_link6", "/panda_link7" };

    public static readonly string[] RosLinkNames =
    { "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7" };


    private string robotNamespace = "franka";
    private string desired_ee_pose_topic;
    private string desired_joint_states_topic;
    private UrdfJointRevolute[] robot_joints;
    private int nb_robots_joints = 7;
    string[] linkNames;



    // Start is called before the first frame update
    void Start()
    {

        ros = ROSConnection.GetOrCreateInstance();

        desired_ee_pose_topic = "ee_target_pose";
        desired_joint_states_topic = robotNamespace + "/desired_joint_state";

        // Get UrdfJointRevolute components instead of ArticulationBody components
        robot_joints = new UrdfJointRevolute[nb_robots_joints];
        m_JointArticulationBodies = new List<ArticulationBody>(GetComponentsInChildren<ArticulationBody>()); // maybe the problem is here 

        var linkName = string.Empty;
        for (var i = 0; i < nb_robots_joints; i++)
        {
            linkName += LinkNames[i];
            robot_joints[i] = panda.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
      
        }

        ros.RegisterPublisher<JointStateMsg>(robotNamespace + "/current_joint_states");
        ros.RegisterPublisher<PoseStampedMsg>("ee_target_pose");
        Debug.Log("Je trouve les informations de mes joints ? ");
        // Invoke these methods after a delay of 1 second.
        Invoke("CurrentJointState", 0.5f);
        Debug.Log("Je trouve les informations de mes joints current ? ");
        Invoke("SendTargetPose", 0.5f);


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
        Debug.Log("Voici le message reçu sous ROS:" + msg);
        // Vérifiez que le message a le bon nombre de positions
        if (msg.position.Length != nb_robots_joints)
        {
            Debug.LogError("Received JointStateMsg with incorrect number of positions");
            return;
        }


        // Set the joint values for every joint
        for (var joint = 1; joint < nb_robots_joints + 1 ; joint++)
        {   

            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target = (float)msg.position[joint - 1] * Mathf.Rad2Deg;
            m_JointArticulationBodies[joint].xDrive = joint1XDrive ;
            Debug.Log("Joint name: " + m_JointArticulationBodies[joint].name + "angle :" + joint1XDrive.target);

        }

    }


    // Update is called once per frame
    void Update()
    {

        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>(desired_joint_states_topic, HandleDesiredJointStateMessage);
        // Votre code existant ici...
        Debug.Log("Position de l'effecteur de fin :" + endEffector.transform.position);
        Debug.Log("Orientation de l'effecteur de fin :" + endEffector.transform.rotation);

    }
}



