using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class ROSUnityTransformPublisher : MonoBehaviour
{
    public string FrameID = "world";
    public string ChildFrameID = "base_link";

    private TransformStampedMsg _transformMessage = new TransformStampedMsg();
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // If the child frame ID is not set, then set it to be the name of this game object.
        if (string.IsNullOrEmpty(ChildFrameID))
        {
            ChildFrameID = this.gameObject.name;
        }
    }

    void Update()
    {
        // Update header
        _transformMessage.header.stamp.sec = (uint)Time.time;
        _transformMessage.header.stamp.nanosec = (uint)(Time.time * 1e9 % 1e9);
        _transformMessage.header.frame_id = FrameID;
        _transformMessage.child_frame_id = ChildFrameID;

        // Update transform
        _transformMessage.transform.translation = transform.position.To<FLU>();
        _transformMessage.transform.rotation = transform.rotation.To<FLU>();

        // Publish transform
        ros.Publish("/tf", _transformMessage);
    }


    void OnValidate()
    {
        if (string.IsNullOrEmpty(ChildFrameID))
        {
            ChildFrameID = name;
        }

        if (string.IsNullOrEmpty(FrameID) && transform.parent)
        {
            FrameID = transform.parent.name;
        }
    }
}

