using System;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class SimpleLIDAR2D : MonoBehaviour
{
    [Header("FOV")]
    public float MinAngleDegrees = 0.0f;
    public float MaxAngleDegrees = 360.0f;
    public float AngleIncrementDegrees = 1.0f;
    public float MinRangeDistanceMeters = 0.1f;
    public float MaxRangeDistanceMeters = 20.0f;

    public int RayCount { get; private set; }
    private Ray[] _rays;
    private RaycastHit[] _raycastHits;
    private float[] _ranges;

    private ROSConnection ros;
    private string rosTopic = "/scan";
    private LaserScanMsg _rosLaserScanMessage = new LaserScanMsg();

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(rosTopic);


        RayCount = Mathf.RoundToInt((MaxAngleDegrees - MinAngleDegrees) / AngleIncrementDegrees);
        _rays = new Ray[RayCount];
        _raycastHits = new RaycastHit[RayCount];
        _ranges = new float[RayCount];
    }

    private void FixedUpdate()
    {
        ScanEnvironment();
        PublishToROS();
    }

    private void ScanEnvironment()
    {
        for (int i = 0; i < RayCount; i++)
        {
            Vector3 axis = new Vector3(0, MinAngleDegrees - AngleIncrementDegrees * i, 0);
            Vector3 direction = Quaternion.Euler(axis) * transform.forward;
            _rays[i] = new Ray(transform.position, direction);
            _ranges[i] = 0;

            if (Physics.Raycast(_rays[i], out _raycastHits[i], MaxRangeDistanceMeters))
            {
                if (_raycastHits[i].distance >= MinRangeDistanceMeters && _raycastHits[i].distance <= MaxRangeDistanceMeters)
                {
                    _ranges[i] = _raycastHits[i].distance;
                    Debug.DrawLine(transform.position, _raycastHits[i].point, Color.green);
                }
            }
        }
    }

    private void PublishToROS()
    {
        _rosLaserScanMessage.header.frame_id = "base_footprint";  
        _rosLaserScanMessage.angle_min = MinAngleDegrees * Mathf.Deg2Rad;
        _rosLaserScanMessage.angle_max = MaxAngleDegrees * Mathf.Deg2Rad;
        _rosLaserScanMessage.angle_increment = AngleIncrementDegrees * Mathf.Deg2Rad;
        _rosLaserScanMessage.time_increment = 0; 
        _rosLaserScanMessage.scan_time = 1.0f / RayCount;
        _rosLaserScanMessage.range_min = MinRangeDistanceMeters;
        _rosLaserScanMessage.range_max = MaxRangeDistanceMeters;
        _rosLaserScanMessage.ranges = _ranges;

        ros.Publish(rosTopic, _rosLaserScanMessage);
    }
}
