using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;

[System.Serializable]
public struct SizeInt
{
    public int rows, cols;

    public SizeInt(int r, int c)
    {
        rows = r;
        cols = c;
    }
}

[System.Serializable]
public struct FoV
{
    public float h, v;

    public FoV(float hFov, float vFov)
    {
        h = hFov;
        v = vFov;
    }
}



public class MiroRGB3DCameraV7 : MonoBehaviour {


    public bool RGBEnabler = false;
    [Tooltip("Field of View (degree)")]
    public FoV FOV;

    [Tooltip("Image resolution (pixel)")]
    public SizeInt Size;

    [Tooltip("Sensor max distance (m)")]
    public float MaxRange;

    [Tooltip("Only for drawing (rgba)")]
    public Color ExtremaPointsColor, ActualPointsColor;

    [Tooltip("Only for drawing (m)")]
    public float PointDrawingScale;

    public float multi_path_noise = 0.3f; //magnitude for the multipath error
    public float max_ray_pass = 0.5f; //distance btw bounce ray and camera
    public float max_reflection_dist = 3f; //max distance of the reflection to consider

    public float max_edge_magn = 0.3f;
    public float max_dist_magn = 0.1f;

    public bool edge_noise = false;
    public bool dist_nois = false;
    public bool multi_path_error = false;

    private Vector3[] PointsToReach;
    private Mesh[] meshArray;

    private Vector3[] LastCapturedPoints;                                       // store xyz info of the last captured point cloud
    private Vector3[] LastCapturedPoints_color;                                 // store RGB info of the last captured point cloud


    private bool initialized = false;
    private GameObject extremalPointsParent, actualPointsParent;

    Mesh meshExtrema, meshActual;
    Material materialExtrema, materialActual;
    
    public bool IsInitialized() { return initialized; }

    void Start()
    {
        meshExtrema = new Mesh();
        meshActual = new Mesh();
        Init();
    }

    void OnValidate()
    {
        initialized = false;
    }

    void Update()
    {
        if (!initialized)
        {
            Init();
        }
    }

    void Init()
    {
        materialExtrema = new Material(Resources.Load<Shader>("GeometryShader"));
        materialExtrema.SetFloat("_Size", PointDrawingScale);
        materialExtrema.SetColor("_Color", ExtremaPointsColor);

        materialActual = new Material(Resources.Load<Shader>("GeometryShader"));
        materialActual.SetFloat("_Size", PointDrawingScale);
        materialActual.SetColor("_Color", ActualPointsColor);

        LastCapturedPoints = new Vector3[0];            // store xyz info
        LastCapturedPoints_color = new Vector3[0];      // store color info

        if (Size.rows <= 0) Debug.LogError("INVALID INPUT for camera vertical resolution. Actual value is " + Size.rows);
        if (Size.cols <= 0) Debug.LogError("INVALID INPUT for camera horizontal resolution. Actual value is " + Size.cols);

        if (MaxRange <= 0) Debug.LogError("INVALID INPUT for camera max visual distance. Actual value is " + MaxRange);

        if (FOV.v < 0) Debug.LogError("INVALID INPUT for camera vertical field of view. Actual value is " + FOV.v);
        if (FOV.h < 0) Debug.LogError("INVALID INPUT for camera horizontal field of view. Actual value is " + FOV.h);

        // change camera
        Camera.main.fieldOfView = FOV.v;

        // z si allontana dal sensore
        // x verso il basso
        // y verso sinistra
        float minX = 0.0f - FOV.v / 2.0f;
        float maxX = 0.0f + FOV.v / 2.0f;

        float minY = 0.0f - FOV.h / 2.0f;
        float maxY = 0.0f + FOV.h / 2.0f;

        float deltaX = (maxX - minX) / (float)Size.rows;
        float deltaY = (maxY - minY) / (float)Size.cols;

        PointsToReach = new Vector3[Size.cols * Size.rows];
                
        for (int r = 0; r < Size.cols; r++)
        {
            for (int c = 0; c < Size.rows; c++)
            {
                Vector3 p = new Vector3(0, 0, MaxRange);
                p = Quaternion.Euler(minX + deltaX * 0.5f + deltaX * c, minY + deltaY * 0.5f + deltaY * r, 0) * p;
                PointsToReach[r * Size.rows + c] = p;
            }
        }

        initialized = true;
        Debug.Log("Camera parameters updated!");
    }


    public SizeInt GetPointcloudSize()
    {
        return Size;      
    }

    public void SwitchRGBCatcher(bool rgbFlag)
    {
        RGBEnabler = rgbFlag;
    }

    public int edge_length = 3;
    public bool flying_pixel = false; // enable the drawing of the rays of the edges of the object with the rays
    public float timeray = 10; // time to leave the rays visible

    public void CalcPointCloud(out Vector3[] pts, out Vector3[] pts_color)
    {
        Init();
        // template: pts[i] = <x,y,z>;
        pts = new Vector3[PointsToReach.Length];        // temporary hitting point xyz container
        // template: pts_color[i] = <r,g,b>;
        pts_color = new Vector3[PointsToReach.Length];  // temporary hitting point color container

        Vector3 worldTarget;
        Ray ray;                                                                                            // 0) create the ray object
        RaycastHit hit;                                                                                     // 1) create RaycastHit object

        float[] ray_distance = new float[PointsToReach.Length];

        for (int i = 0; i < PointsToReach.Length; i++)  // interate the entire To-Be-Reach points
        {
            // initialize the world target
            worldTarget = transform.TransformPoint(PointsToReach[i]);

            // Assign a new Ray to Ray object: new Ray(origin, direction)
            // +++ Origin: RGB3D camera
            // +++ Direction: the vector from the Origin to World Target
            ray = new Ray(transform.position, worldTarget - transform.position);                            // 0_1) Assign a new Ray

            if (Physics.Raycast(ray, out hit))  // if hit
            {
                // (A) set xyz info
                ray_distance[i] = hit.distance;

                pts[i] = transform.InverseTransformPoint(hit.point);

                if (pts[i].magnitude > MaxRange)
                {
                    pts[i] = new Vector3(0, 0, 0);
                }

                // for multipath errors
                if (multi_path_error)
                {
                    // Find the line from the camera to the point hit in ground.
                    Vector3 incomingVec = hit.point - transform.position;
                    // Use the point's normal to calculate the reflection vector.
                    Vector3 reflectVec = Vector3.Reflect(incomingVec, hit.normal);
                    //reflected ray from the first hit point 
                    Ray refl_ray = new Ray(hit.point, reflectVec);
                    RaycastHit hit2;
                    if (Physics.Raycast(refl_ray, out hit2, max_reflection_dist))
                    {
                        // distance from the first hit point to the second hit point
                        Vector3 first_bounce_vect = hit.point - hit2.point;
                        // line of the second reflection
                        Vector3 returnVec = Vector3.Reflect(first_bounce_vect, hit2.normal);
                        //ray of the coming back
                        Ray return_ray = new Ray(hit2.point, returnVec);
                        // distance btw the coming back ray and the camera
                        float bounc_distance = (hit2.point - transform.position).magnitude * Mathf.Tan(Vector3.Angle(hit2.point - transform.position, returnVec) * Mathf.Deg2Rad);
                        //Vector3 bounc_distance = Vector3.Cross(return_ray.direction, transform.position-return_ray.origin);

                        float sigma_square = Mathf.Pow(max_reflection_dist / 6, 2);
                        float gaus_error = (1 / Mathf.Sqrt(2.0f * Mathf.PI * sigma_square)) * Mathf.Exp((-0.5f / sigma_square) * (Mathf.Pow(first_bounce_vect.magnitude - max_reflection_dist / 2, 2)));

                        float gaus_modifiy = (max_reflection_dist * Mathf.Exp(-0.5f * Mathf.Pow(first_bounce_vect.magnitude - (max_reflection_dist / 4), 2) / sigma_square)) / (1 + ((first_bounce_vect.magnitude - (max_reflection_dist / 4)) * 4) / sigma_square);
                        //if (bounc_distance < max_ray_pass)
                        //pts[i] = pts[i] + incomingVec.normalized * gaus_error * multi_path_noise;
                        pts[i] = pts[i] + pts[i].normalized * gaus_error * (multi_path_noise * max_reflection_dist);
                    }
                }

                //adding noise distance
                if (dist_nois)
                {
                    pts[i][2] = pts[i][2] * (1 + pts[i].magnitude / MaxRange * UnityEngine.Random.Range(0f, max_dist_magn));
                }

                // (B) set RGB color info
                if (RGBEnabler) // if enable the RGBEnabler
                {
                    Renderer rend_i = hit.transform.GetComponent<Renderer>();                                   // 3) get the Renderer attribute of the hit
                    MeshCollider meshCollider_i = hit.collider as MeshCollider;                                 // 4) get the MeshCollider attribute of the hit

                    if (rend_i == null || rend_i.sharedMaterial == null                                         // 5) if successfully init Renderer, MeshCollider of the hit
                                       || rend_i.sharedMaterial.mainTexture == null
                                       || meshCollider_i == null)
                    {
                        Debug.Log("WARNING: The Renderer or MeshCollider of the captured Object fails to load.");
                        Debug.Log("WARNING: The point cloud of that object will be all in black color");
                        pts_color[i] = new Vector3(0.0f, 0.0f, 0.0f);                                           // no RGB info, all-black point cloud
                    }
                    else
                    {
                        Texture2D tex_i = rend_i.material.mainTexture as Texture2D;                             // 6) get the mainTexture of the hit
                        Vector2 pixelUV_i = hit.textureCoord;                                                   // 7) get the UV coordinate of the texture
                        pixelUV_i.x *= tex_i.width;                                                             //    set the UV.x
                        pixelUV_i.y *= tex_i.height;                                                            //    set the UV.y
                        Vector2 tiling_i = rend_i.material.mainTextureScale;                                    //    get the tiling scale

                        Color color_i = tex_i.GetPixel(Mathf.FloorToInt(pixelUV_i.x * tiling_i.x),
                                                       Mathf.FloorToInt(pixelUV_i.y * tiling_i.y));             // 8) get the pixel object's color object of the texture
                        pts_color[i] = new Vector3(color_i.r * 255.0f,                                          // 9) distill and store R,G,B info from the color object
                                                   color_i.g * 255.0f,
                                                   color_i.b * 255.0f);            
                    }
                }
            }
            else // if not hit
            {
                // (A) store XYZ info
                pts[i] = Vector3.zero;
                // (B) store RGB info
                if (RGBEnabler) // if enabler RGBEnabler
                {
                    pts_color[i] = new Vector3(0.0f, 0.0f, 0.0f);                                               // no RGB info
                }
            }
        }

        LastCapturedPoints = pts;                                                                               // store captured xyz info
        if (RGBEnabler)
        {
            LastCapturedPoints_color = pts_color;                                                               // store captured rgb info
        }

        Ray ray1;
        Vector3 ray2;
        // error of flying pixels
        if (flying_pixel)
        {
            // check if the distances of the neighborn are bigger than a threshold
            for (int r = 1; r < Size.cols; r++)
            {
                for (int c = 1; c < Size.rows; c++)
                {
                    // look the lower edge
                    // check if the previous length in the r-th row is bigger than a threshold
                    if (ray_distance[r * Size.rows + c] > edge_length + ray_distance[r * Size.rows + c - 1])
                    {
                        worldTarget = transform.TransformPoint(PointsToReach[r * Size.rows + c]);
                        ray = new Ray(transform.position, worldTarget - transform.position);

                        Debug.DrawRay(ray.origin, ray.direction * ray_distance[r * Size.rows + c], Color.red, duration: timeray);

                        //AddLine(worldTarget - transform.position, Color.red);

                        // rewrite the point out of the edge of the object with a random position beetwen the distance of the front object and the rear object
                        pts[r * Size.rows + c] = transform.TransformPoint(pts[r * Size.rows + c]);
                        pts[r * Size.rows + c] = transform.InverseTransformPoint(ray.GetPoint(UnityEngine.Random.Range(ray_distance[r * Size.rows + c], ray_distance[r * Size.rows + c - 1])));
                    }

                    // look the upper edge
                    // check if the previous length in the r-th row is lower than a threshold
                    if (ray_distance[r * Size.rows + c - 1] > edge_length + ray_distance[r * Size.rows + c])
                    {
                        worldTarget = transform.TransformPoint(PointsToReach[r * Size.rows + c - 1]);
                        ray = new Ray(transform.position, worldTarget - transform.position);
                        Debug.DrawRay(ray.origin, ray.direction * ray_distance[r * Size.rows + c - 1], Color.black, duration: timeray);

                        //AddLine(worldTarget - transform.position, Color.black);

                        // rewrite the point out of the edge of the object with a random position beetwen the distance of the front object and the rear object
                        pts[r * Size.rows + c - 1] = transform.TransformPoint(pts[r * Size.rows + c - 1]);
                        pts[r * Size.rows + c - 1] = transform.InverseTransformPoint(ray.GetPoint(UnityEngine.Random.Range(ray_distance[r * Size.rows + c], ray_distance[r * Size.rows + c - 1])));
                    }

                    //look the right edge
                    // check if the previous length in the c-th row is bigger than a threshold
                    if (ray_distance[r * Size.rows + c] > edge_length + ray_distance[(r - 1) * Size.rows + c])
                    {
                        worldTarget = transform.TransformPoint(PointsToReach[r * Size.rows + c]);
                        ray = new Ray(transform.position, worldTarget - transform.position);
                        Debug.DrawRay(ray.origin, ray.direction * ray_distance[r * Size.rows + c], Color.yellow, duration: timeray);

                        //AddLine(worldTarget - transform.position, Color.yellow);

                        // rewrite the point out of the edge of the object with a random position beetwen the distance of the front object and the rear object
                        pts[r * Size.rows + c] = transform.TransformPoint(pts[r * Size.rows + c]);
                        pts[r * Size.rows + c] = transform.InverseTransformPoint(ray.GetPoint(UnityEngine.Random.Range(ray_distance[r * Size.rows + c], ray_distance[(r - 1) * Size.rows + c])));
                    }

                    //look the left edge
                    // check if the previous length in the c-th row is bigger than a threshold
                    if (ray_distance[(r - 1) * Size.rows + c] > edge_length + ray_distance[r * Size.rows + c])
                    {
                        worldTarget = transform.TransformPoint(PointsToReach[(r - 1) * Size.rows + c]);
                        ray = new Ray(transform.position, worldTarget - transform.position);
                        Debug.DrawRay(ray.origin, ray.direction * ray_distance[(r - 1) * Size.rows + c], Color.cyan, duration: timeray);

                        //AddLine(worldTarget - transform.position, Color.cyan);

                        // rewrite the point out of the edge of the object with a random position beetwen the distance of the front object and the rear object
                        pts[(r - 1) * Size.rows + c] = transform.TransformPoint(pts[(r - 1) * Size.rows + c]);
                        pts[(r - 1) * Size.rows + c] = transform.InverseTransformPoint(ray.GetPoint(UnityEngine.Random.Range(ray_distance[r * Size.rows + c], ray_distance[(r - 1) * Size.rows + c])));
                    }

                }
            }
        }
    }
    public void DrawExtrema(bool visibility)
    {

        if (extremalPointsParent == null)
        {
            extremalPointsParent = new GameObject();
            extremalPointsParent.name = "Extrema";
            extremalPointsParent.transform.parent = transform;
            extremalPointsParent.transform.localPosition = Vector3.zero;
            extremalPointsParent.transform.localRotation = Quaternion.identity;
            extremalPointsParent.AddComponent<MeshFilter>();
            MeshRenderer mr = extremalPointsParent.AddComponent<MeshRenderer>();
            mr.receiveShadows = false;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            
        }
        
        if (visibility)
        {
            int max = 60000;
            Vector3[] vertices = new Vector3[max];
            int[] indices = new int[max];
            for (int i = 0; i < max; i++)
            {
                int e = UnityEngine.Random.Range(0, PointsToReach.Length);
                vertices[i] = transform.InverseTransformPoint(PointsToReach[e]);
                indices[i] = i;
            }

            meshExtrema.vertices = vertices;
            meshExtrema.SetIndices(indices, MeshTopology.Points, 0);            

        } else
        {
            meshExtrema.Clear();
        }

        meshExtrema.RecalculateBounds();

        extremalPointsParent.GetComponent<MeshFilter>().mesh = meshExtrema;
        extremalPointsParent.GetComponent<MeshRenderer>().material = materialExtrema;
    }
    
    public void DrawLastCapturedPoints(bool visibility)
    {

        if (actualPointsParent == null)
        {
            actualPointsParent = new GameObject();
            actualPointsParent.name = "LastCaptured";
            actualPointsParent.transform.parent = transform;
            actualPointsParent.transform.localPosition = Vector3.zero;
            actualPointsParent.transform.localRotation = Quaternion.identity;
            actualPointsParent.AddComponent<MeshFilter>();
            MeshRenderer mr = actualPointsParent.AddComponent<MeshRenderer>();
            mr.receiveShadows = false;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

        }

        if (visibility)
        {
            int max = 60000;
            Vector3[] vertices = new Vector3[max];
            int[] indices = new int[max];
            for (int i = 0; i < max; i++)
            {
                int e = UnityEngine.Random.Range(0, LastCapturedPoints.Length);
                vertices[i] = transform.InverseTransformPoint(LastCapturedPoints[e]);
                indices[i] = i;
            }

            meshActual.vertices = vertices;
            meshActual.SetIndices(indices, MeshTopology.Points, 0);

        }
        else
        {
            meshActual.Clear();
        }

        meshActual.RecalculateBounds();

        actualPointsParent.GetComponent<MeshFilter>().mesh = meshActual;
        actualPointsParent.GetComponent<MeshRenderer>().material = materialActual;
    }

    /*
    // RGB Info Packer

    // 1. Pack RGB to Int: WARNING: Overflow issue with Point Cloud Library
    // Pack each separate RGB color info stored in the Vector3 container into an int32 for PCD format
    public int[] PackRGBInfoToInt()
    {
        int[] packedRGB_int = new int[LastCapturedPoints_color.Length];

        for (int i = 0; i < LastCapturedPoints_color.Length; i++)
        {
            int r_i = (int)LastCapturedPoints_color[i].x;               // red info
            int g_i = (int)LastCapturedPoints_color[i].y;               // green info
            int b_i = (int)LastCapturedPoints_color[i].z;               // blue info
            int rgb = ((int)r_i << 16 | (int)g_i << 8 | (int)b_i);      // use bit operation bit-store R,G,B info
            
            packedRGB_int[i] = rgb;

        }

        return packedRGB_int;                                           // return bit storage packed RGB info
    }*/


    // 2. Pack RGB to Float: Ideal datastructure for bit-store +++ BEST CHOICE +++
    // Notes: Due to the type-safe scheme in C#, we need to enbale unsafe programming option in Unity and Visual Studio
    //        Then, we can cast bit-by-bit uint to float
    // Pack each separate RGB color info stored in the Vector3 container into an float32 for PCD format
    public float[] PackRGBInfoToFloat()
    {
        float[] packedRGB_float = new float[LastCapturedPoints_color.Length];

        for (int i = 0; i < LastCapturedPoints_color.Length; i++)
        {
            uint r_i = (uint)LastCapturedPoints_color[i].x;               // red info
            uint g_i = (uint)LastCapturedPoints_color[i].y;               // green info
            uint b_i = (uint)LastCapturedPoints_color[i].z;               // blue info
            uint rgb = ((uint)r_i << 16 | (uint)g_i << 8 | (uint)b_i);      // use bit operation bit-store R,G,B info
            packedRGB_float[i] = 1.0f;
            unsafe
            {
                uint* uptr = &rgb;
                packedRGB_float[i] = *((float*)uptr);
            }
        }

        return packedRGB_float;                                           // return bit storage packed RGB info
    }

    public void CreatePCD(string filename)
    {
        StreamWriter writer = new StreamWriter(filename, false);

        if (RGBEnabler) // Write PCD format, if enable RGBEnabler
        {
            float[] packedRGB_float = PackRGBInfoToFloat();                   // packed (int)R, (int)G, (int)B info into (int)RGB

            writer.WriteLine("# .PCD v.7 - Point Cloud Data file format");
            writer.WriteLine("VERSION .7");
            writer.WriteLine("FIELDS x y z rgb");
            writer.WriteLine("SIZE 4 4 4 4");
            writer.WriteLine("TYPE F F F F");
            writer.WriteLine("COUNT 1 1 1 1");
            writer.WriteLine("WIDTH " + LastCapturedPoints.Length.ToString());
            writer.WriteLine("HEIGHT 1");
            writer.WriteLine("VIEWPOINT 0 0 0 1 0 0 0");
            writer.WriteLine("POINTS " + LastCapturedPoints.Length.ToString());
            writer.WriteLine("DATA ascii");

            for (int i = 0; i < LastCapturedPoints.Length; i++)
            {
                float fx = LastCapturedPoints[i].x;
                float fy = LastCapturedPoints[i].y;
                float fz = LastCapturedPoints[i].z;

                if (System.Math.Abs(fx) <= 0.00000f && System.Math.Abs(fy) <= 0.00000f && System.Math.Abs(fz) <= 0.00000f)
                {
                    writer.Write("nan");
                    writer.Write(" ");
                    writer.Write("nan");
                    writer.Write(" ");
                    writer.Write("nan");
                    writer.Write(" ");
                    writer.WriteLine("nan");
                }
                else
                {
                    writer.Write(fx.ToString("0.00000"));
                    writer.Write(" ");
                    writer.Write(fy.ToString("0.00000"));
                    writer.Write(" ");
                    writer.Write(fz.ToString("0.00000"));
                    writer.Write(" ");
                    writer.WriteLine(packedRGB_float[i].ToString());
                }
            }
        }
        else          //Write PCD format, if not enable RGBEnabler
        {
            writer.WriteLine("# .PCD v.7 - Point Cloud Data file format");
            writer.WriteLine("VERSION .7");
            writer.WriteLine("FIELDS x y z");
            writer.WriteLine("SIZE 4 4 4");
            writer.WriteLine("TYPE F F F");
            writer.WriteLine("COUNT 1 1 1");
            writer.WriteLine("WIDTH " + LastCapturedPoints.Length.ToString());
            writer.WriteLine("HEIGHT 1");
            writer.WriteLine("VIEWPOINT 0 0 0 1 0 0 0");
            writer.WriteLine("POINTS " + LastCapturedPoints.Length.ToString());
            writer.WriteLine("DATA ascii");

            for (int i = 0; i < LastCapturedPoints.Length; i++)
            {
                float fx = LastCapturedPoints[i].x;
                float fy = LastCapturedPoints[i].y;
                float fz = LastCapturedPoints[i].z;

                if (System.Math.Abs(fx) <= 0.00000f && System.Math.Abs(fy) <= 0.00000f && System.Math.Abs(fz) <= 0.00000f)
                {
                    writer.Write("nan");
                    writer.Write(" ");
                    writer.Write("nan");
                    writer.Write(" ");
                    writer.WriteLine("nan");
                }
                else
                {
                    writer.Write(fx.ToString("0.00000"));
                    writer.Write(" ");
                    writer.Write(fy.ToString("0.00000"));
                    writer.Write(" ");
                    writer.WriteLine(fz.ToString("0.00000"));
                }
            }
        }

        writer.Close();
    }
}
