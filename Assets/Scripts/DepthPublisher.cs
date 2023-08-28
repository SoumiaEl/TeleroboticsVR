using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class DepthPublisher : MonoBehaviour
{
    public MiroRGB3DCameraV7 miroDepthCamera;

    private ROSConnection ros;
    private string depthTopic = "/camera/depth_registered/image_raw";

    private Texture2D depthTexture;
    private RenderTexture renderTexture;
    private Camera depthCamera;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        int imageWidth = miroDepthCamera.Size.cols;
        int imageHeight = miroDepthCamera.Size.rows;

        depthTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);

        depthCamera = miroDepthCamera.transform.Find("Depth Camera").GetComponent<Camera>();
        if (depthCamera == null)
        {
            Debug.LogError("No Camera component found on Depth Camera child.");
            return;
        }
        depthCamera.targetTexture = renderTexture;

        ros.RegisterPublisher<ImageMsg>(depthTopic);
    }

    void LateUpdate()
    {
        PublishDepthImage();
    }

    private void PublishDepthImage()
    {
        if (depthCamera == null) return;

        depthCamera.Render();

        RenderTexture.active = renderTexture;
        depthTexture.ReadPixels(new Rect(0, 0, miroDepthCamera.Size.cols, miroDepthCamera.Size.rows), 0, 0);
        depthTexture.Apply();

        ImageMsg depthMsg = ConvertToROSImage(depthTexture);
        ros.Publish(depthTopic, depthMsg);
    }

    private ImageMsg ConvertToROSImage(Texture2D texture)
    {
        ImageMsg msg = new ImageMsg();
        msg.data = texture.GetRawTextureData();
        msg.width = (uint)texture.width;
        msg.height = (uint)texture.height;
        msg.encoding = (texture.format == TextureFormat.RGB24) ? "rgb8" : "mono8"; //32FC1 for RTAB-MAP
        msg.step = (uint)(texture.width * (texture.format == TextureFormat.RGB24 ? 3 : 4));



        // Adding logs to help with debugging
        Debug.Log("Texture Format: " + texture.format.ToString());
        Debug.Log("Image Width: " + msg.width);
        Debug.Log("Image Height: " + msg.height);
        Debug.Log("Image Encoding: " + msg.encoding);
        Debug.Log("Image Step: " + msg.step);
        Debug.Log("Raw Data Length: " + msg.data.Length);


        return msg;
    }
}
