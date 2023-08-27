using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter;

public class HandleROSVelocity : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    GameObject target;

    // Articulation Bodies
    List<ArticulationBody> m_JointArticulationBodies;


    // Topic to publish joint velocities
    private string joint_velocity_topic = "joint_velocity_command";
    private string ee_target_velocity_topic = "/ee_target_velocity";


    private Vector3 lastPosition;
    private float lastTime;
    private UrdfJointRevolute[] robot_joints;
    private int nb_robots_joints = 7;

    private TwistMsg targetVelocityMsg = new TwistMsg();

    private float timeSinceLastMovement = 0f;
    private float movementThreshold = 0.01f;  // Ajustez cette valeur en fonction de vos besoins


    public static readonly string[] RosLinkNames =
    { "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7" };

    private string robotNamespace = "franka";
    private JointStateMsg lastJointStateMsg = null;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        m_JointArticulationBodies = new List<ArticulationBody>(GetComponentsInChildren<ArticulationBody>());


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
            var drive = m_JointArticulationBodies[i+1].xDrive;
            drive.target = initialJointPositions[i] * Mathf.Rad2Deg; // Convert to degrees as Unity uses degrees for rotation
            m_JointArticulationBodies[i+1].xDrive = drive; // Set the target rotation for the joint

        }

        // Ajout d'un log pour vérifier le driveType de chaque joint après l'appel de SwitchToVelocityControl
        foreach (var jointBody in m_JointArticulationBodies)
        {
            Debug.Log($"Vérification - Joint: {jointBody.name}, Drive Type: {jointBody.xDrive.driveType}");
        }

        SwitchToVelocityControl();

        // Ajout d'un log pour vérifier le driveType de chaque joint après l'appel de SwitchToVelocityControl
        foreach (var jointBody in m_JointArticulationBodies)
        {
            Debug.Log($"Vérification - Joint: {jointBody.name}, Drive Type: {jointBody.xDrive.driveType}");
        }

       
      
       
        ros.Subscribe<JointStateMsg>(robotNamespace + "/desired_joint_velocity", HandleDesiredJointVelocityMessage);
        ros.RegisterPublisher<TwistMsg>(robotNamespace + ee_target_velocity_topic);


        lastPosition = target.transform.position;
        lastTime = Time.time;

       

    }

    void Update()
    {
        // Calculer la vitesse
        float deltaTime = Time.time - lastTime;
        Vector3 currentVelocity = (target.transform.position - lastPosition) / deltaTime;

        // Mise à jour de lastPosition et lastTime pour la prochaine fois
        lastPosition = target.transform.position;
        lastTime = Time.time;

        // Créer le message TwistMsg
        targetVelocityMsg.linear.x = currentVelocity.x;
        targetVelocityMsg.linear.y = currentVelocity.y;
        targetVelocityMsg.linear.z = currentVelocity.z;

        Debug.Log($"Current Target Velocity (Unity): X: {currentVelocity.x}, Y: {currentVelocity.y}, Z: {currentVelocity.z}");


        if (currentVelocity.magnitude > movementThreshold)
        {
            Debug.Log("Target is moving. Switching to Velocity Control.");
            timeSinceLastMovement = 0f;
            SwitchToVelocityControl();
        }
        else
        {
            Debug.Log("Target is stationary. Switching to Position Control.");
            timeSinceLastMovement += Time.deltaTime;
            if (timeSinceLastMovement > 2f)  // Si la target est immobile depuis plus de 2 secondes
            {
                SwitchToPositionControlWithCurrentPositions();
            }
        }

        // Si vous avez besoin de vitesse angulaire, mettez à jour targetVelocityMsg.angular également
        if (m_JointArticulationBodies[0].xDrive.driveType == ArticulationDriveType.Velocity)
        {
            // Publier le message
            ros.Publish(robotNamespace + ee_target_velocity_topic, targetVelocityMsg);
        }
    }



    void HandleDesiredJointVelocityMessage(JointStateMsg msg)
    {
        // Update the last joint state message
        lastJointStateMsg = msg;

        if (msg == null)
        {
            Debug.Log("Received null JointStateMsg");
            return;
        }

        // Check that the message has the correct number of velocities
        if (msg.velocity.Length != RosLinkNames.Length)
        {
            Debug.Log("Received JointStateMsg with incorrect number of velocities");
            return;
        }

        // Check if all velocities are zero
        bool allZero = true;
        foreach (var v in msg.velocity)
        {
            if (Mathf.Abs((float)v) > 1e-6)
            {
                allZero = false;
                break;
            }
        }

        if (allZero)
        {
            SwitchToPositionControlWithCurrentPositions();
        }
        else
        {
            SwitchToVelocityControl();

            // Set the joint target velocities
            for (var joint = 0; joint < RosLinkNames.Length; joint++)
            {
                var jointDrive = m_JointArticulationBodies[joint + 1].xDrive;
                Debug.Log($"Received Velocity for {RosLinkNames[joint]}: {msg.velocity[joint]} rad/s");
                jointDrive.target = (float)msg.velocity[joint];
                m_JointArticulationBodies[joint + 1].xDrive = jointDrive;
                Debug.Log("Joint name: " + m_JointArticulationBodies[joint + 1].name + " target velocity :" + jointDrive.target);
            }
        }

    }

    void SwitchToPositionControlWithCurrentPositions()
    {
        foreach (var jointBody in m_JointArticulationBodies)
        {
            var drive = jointBody.xDrive;
            drive.driveType = ArticulationDriveType.Target;
            jointBody.xDrive = drive;
            Debug.Log($"Switched {jointBody.name} to Position Control. Drive Type: {jointBody.xDrive.driveType}");
  
        }
    }

    private void SwitchToVelocityControl()
    {
        foreach (var jointBody in m_JointArticulationBodies)
        {
            var drive = jointBody.xDrive;
            // Vérifiez si le mode est déjà TargetVelocity
            if (drive.driveType == ArticulationDriveType.Velocity)
                continue;
            



            // Log avant de changer le driveType
            Debug.Log($"Avant le changement - Joint: {jointBody.name}, Drive Type: {drive.driveType}");

            drive.driveType = ArticulationDriveType.Velocity; // Switch to velocity control                                                 
            jointBody.xDrive = drive;

            // Log après le changement
            Debug.Log($"Après le changement - Joint: {jointBody.name}, Drive Type: {drive.driveType}");
            Debug.Log($"Switched {jointBody.name} to Velocity Control. Drive Type: {jointBody.xDrive.driveType}");
        }
    }
    }
