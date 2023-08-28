using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class RGBPublisher : MonoBehaviour
{
    public MiroRGB3DCameraV7 miroRGBCamera;

    private ROSConnection ros;
    private string rgbTopic = "/camera/rgb/image_rect_color";

    private Texture2D rgbTexture;
    private RenderTexture renderTexture;
    private Camera rgbCamera;
    private string cameraInfoTopic = "/camera/rgb/camera_info";


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        int imageWidth = miroRGBCamera.Size.cols;
        int imageHeight = miroRGBCamera.Size.rows;

        rgbTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);

        rgbCamera = miroRGBCamera.transform.Find("RGB Camera").GetComponent<Camera>();
        if (rgbCamera == null)
        {
            Debug.LogError("No Camera component found on RGB Camera child.");
            return;
        }
        rgbCamera.targetTexture = renderTexture;

        ros.RegisterPublisher<ImageMsg>(rgbTopic);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);

    }

    void LateUpdate()
    {
        PublishRGBImage();
        PublishCameraInfo();
    }
    private void PublishCameraInfo()
    {
        CameraInfoMsg cameraInfo = new CameraInfoMsg
        {
            width = (uint)rgbCamera.pixelWidth,
            height = (uint)rgbCamera.pixelHeight 
        };

        ros.Publish(cameraInfoTopic, cameraInfo);
    }
    private void PublishRGBImage()
    {
        if (rgbCamera == null) return;

        rgbCamera.Render();

        RenderTexture.active = renderTexture;
        rgbTexture.ReadPixels(new Rect(0, 0, miroRGBCamera.Size.cols, miroRGBCamera.Size.rows), 0, 0);
        rgbTexture.Apply();

        ImageMsg rgbMsg = ConvertToROSImage(rgbTexture);
        ros.Publish(rgbTopic, rgbMsg);
    }

    private ImageMsg ConvertToROSImage(Texture2D texture)
    {
        ImageMsg msg = new ImageMsg();
        msg.data = texture.GetRawTextureData();
        msg.width = (uint)texture.width;
        msg.height = (uint)texture.height;
        msg.encoding = (texture.format == TextureFormat.RGB24) ? "rgb8" : "mono8";
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
