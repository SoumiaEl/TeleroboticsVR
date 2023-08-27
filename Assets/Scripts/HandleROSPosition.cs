using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Std;
using RosMessageTypes.TeleoperationVr;

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

    // Articulation Bodies
    List <ArticulationBody> m_JointArticulationBodies;

    public static readonly string[] LinkNames =
        { "world/panda_link0/panda_link1", "world/panda_link0/panda_link1/panda_link2", "world/panda_link0/panda_link1/panda_link2/panda_link3", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6", "world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7" };

    public static readonly string[] RosLinkNames =
    { "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7" };


    private string robotNamespace = "franka";
    private string desired_ee_pose_topic;
    private string desired_joint_states_topic;
    private UrdfJointRevolute[] robot_joints;
    private int nb_robots_joints = 7;
    private float distanceThreshold = 0.0005f;
    // Add this at the class level
    private JointStateMsg lastJointStateMsg = null;

    //For message

    private string positionTopicHand = "unity_position_user"; // Nom du topic où nous allons publier
    private string positionTopicEndEffector = "unity_position_effector"; // Nom du topic où nous allons publier
    private double previousTime;
    private Vector3 previousPositionHand;
    private Vector3 previousPositionEndEffector;



    // Start is called before the first frame update
    void Start()
    {

        ros = ROSConnection.GetOrCreateInstance();
        previousTime = Time.time;
        previousPositionHand = virtualHand.transform.position;
        previousPositionEndEffector = endEffector.transform.position;


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


        ros.RegisterPublisher<ObjectDataMsg>(positionTopicHand);
        ros.RegisterPublisher<ObjectDataMsg>(positionTopicEndEffector);
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
        for (var joint = 0; joint < nb_robots_joints ; joint++)
        {   

            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target = (float)msg.position[joint] * Mathf.Rad2Deg;
            m_JointArticulationBodies[joint].xDrive = joint1XDrive ;
            Debug.Log("Joint name: " + m_JointArticulationBodies[joint].name + "angle :" + joint1XDrive.target);

        }

    }


    // Update is called once per frame
    void Update()
    {

        double currentTime = Time.time;
        double deltaTime = currentTime - previousTime;
        float deltaTimeFloat = (float)deltaTime;

        // Get the position of the virtual hand and the robot
        Vector3 handPosition = virtualHand.transform.position;
        Vector3 robotPosition = endEffector.transform.position;

        Vector3 velocityHand = new Vector3(
            (handPosition.x - previousPositionHand.x) / deltaTimeFloat,
            (handPosition.y - previousPositionHand.y) / deltaTimeFloat,
            (handPosition.z - previousPositionHand.z) / deltaTimeFloat
        );

        Vector3 velocityEndEffector = new Vector3(
            (robotPosition.x - previousPositionEndEffector.x) / deltaTimeFloat,
            (robotPosition.y - previousPositionEndEffector.y) / deltaTimeFloat,
            (robotPosition.z - previousPositionEndEffector.z) / deltaTimeFloat
        );

        // Publier les données pour la main virtuelle
        var handMessage = new ObjectDataMsg
        {
            object_name = "target",
            timestamp = currentTime,
            position = handPosition.To<FLU>(),
            velocity = velocityHand.To<FLU>() // Convertir en coordonnées ROS (FLU)
        };
        ros.Publish(positionTopicHand, handMessage);

        // Publier les données pour l'effecteur final
        var endEffectorMessage = new ObjectDataMsg
        {
            object_name = "end_effector",
            timestamp = currentTime,
            position = robotPosition.To<FLU>(),
            velocity = velocityEndEffector.To<FLU>() // Convertir en coordonnées ROS (FLU)
        };
        ros.Publish(positionTopicEndEffector, endEffectorMessage);

        // Mettre à jour les valeurs précédentes pour le prochain calcul
        previousTime = currentTime;
        previousPositionHand = handPosition;
        previousPositionEndEffector = robotPosition;



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

    }
}



