using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;




public class HandleROSPosition : MonoBehaviour
{
    ROSConnection ros;
    // Drag the virtual hand object here in the Unity editor
    public GameObject virtualHand;
    public GameObject base_link;
    public GameObject endEffector;

    private string robotNamespace = "franka";
    private string desired_ee_pose_topic;
    private string desired_joint_states_topic;
    private List<ArticulationBody> robot_joints;
    private int nb_robots_joints = 8;



    // Start is called before the first frame update
    void Start()
    {

        ros = ROSConnection.GetOrCreateInstance();

        desired_ee_pose_topic = "ee_target_pose";
        desired_joint_states_topic = robotNamespace + "/desired_joint_state";

        robot_joints = new List<ArticulationBody>(GetComponentsInChildren<ArticulationBody>());

        Debug.Log(robot_joints.Count);

        ros.RegisterPublisher<JointStateMsg>(robotNamespace + "/current_joint_states");
        ros.RegisterPublisher<PoseStampedMsg>("ee_target_pose");

        // Invoke these methods after a delay of 1 second.
        Invoke("CurrentJointState", 1);
        Invoke("SendTargetPose", 1);

        Vector3 rosPosition = ConvertUnityToRosPositionCoordinate(virtualHand.transform.position);
        Quaternion rosOrientation = ConvertUnityToRosOrientationCoordinate(virtualHand.transform.rotation);

        Debug.Log("Initial Position in ROS coordinates: " + rosPosition);
        Debug.Log("Initial Orientation in ROS coordinates: " + rosOrientation);


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

            jointStateMsg.name[i] = robot_joints[i].name;
            jointStateMsg.velocity[i] = robot_joints[i].velocity[0];

            //jointStateMsg.effort[i] = effort_for_joint[i];      // If you can get joint efforts

            if (robot_joints[i].jointType == ArticulationJointType.FixedJoint)
            {
                jointStateMsg.position[i] = 0;
            }
            else
            {
                jointStateMsg.position[i] = robot_joints[i].jointPosition[0];
            }
            Debug.Log("Position du Joint" + i + ":" + jointStateMsg.position[i]);

        }

        Debug.Log(jointStateMsg);
        ros.Publish(robotNamespace + "/current_joint_states", jointStateMsg);

        Debug.Log("Test Position de l'effecteur :" + endEffector.transform.position);
        Debug.Log("Test Rotation de l'effecteur :" + endEffector.transform.rotation);

    }

    void SendTargetPose()
    {

      
        // Obtenir la position et l'orientation de la main virtuelle
        Vector3 unityPosition = endEffector.transform.InverseTransformPoint(virtualHand.transform.position);
        Quaternion unityOrientation = Quaternion.Inverse(endEffector.transform.rotation) * virtualHand.transform.rotation;

        Quaternion rosOrientation = ConvertUnityToRosOrientationCoordinate(unityOrientation);
        Vector3 rosPosition = ConvertUnityToRosPositionCoordinate(unityPosition);

        // Convertir les coordonnées de Unity en coordonnées ROS
        //rosPosition = ConvertUnityToRosPositionCoordinate(rosPosition);
        //rosOrientation = ConvertUnityToRosOrientationCoordinate(rosOrientation);


        // Créer un QuaternionMsg à partir de l'orientation ROS normalisée
        QuaternionMsg normalizedQuaternionMsg = new QuaternionMsg
        {
            x = rosOrientation.x,
            y = rosOrientation.y,
            z = rosOrientation.z,
            w = rosOrientation.w
        };

        // Créer un PoseMsg avec la position et l'orientation ROS
        var poseMsg = new PoseMsg
        {
            position = new PointMsg
            {
                x = rosPosition.x,
                y = rosPosition.y,
                z = rosPosition.z
            },
            orientation = normalizedQuaternionMsg
        };

        // Créer un PoseStampedMsg et le publier
        var poseStampedMsg = new PoseStampedMsg
        {
            header = new HeaderMsg { frame_id = "panda_link0" },
            pose = poseMsg
        };

        Debug.Log("Position souhaitée : " + poseStampedMsg);

        ros.Publish(desired_ee_pose_topic, poseStampedMsg);
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

        // Mettez à jour les positions cibles pour chaque joint
        for (int i = 1; i < nb_robots_joints; i++)
        {
            var drive = robot_joints[i].xDrive;
            drive.target = (float)msg.position[i];
            robot_joints[i].xDrive = drive;
        }
    }

    Vector3 ConvertUnityToRosPositionCoordinate(Vector3 unityPosition)
    {
        return new Vector3(-unityPosition.z, -unityPosition.x, unityPosition.y);
    }


    Quaternion ConvertUnityToRosOrientationCoordinate(Quaternion unityQuaternion)
    {
        return new Quaternion(unityQuaternion.y, unityQuaternion.z, -unityQuaternion.x, -unityQuaternion.w);
    }


    // Update is called once per frame
    void Update()
    {
        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>(desired_joint_states_topic, HandleDesiredJointStateMessage);
        Debug.Log("Position de l'end effecteur:" + endEffector.transform.position);

    }
}


/*
 Position 
x : 0.156
y: -0.263
z : 0.29

Rotation 
x: 0.00027
y : 0.00079,
z  -1.00000 
w : 0.00000

 */