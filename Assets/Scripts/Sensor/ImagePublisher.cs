using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


/// <summary>
///     This script publishes camera view
///     as compressed image 
/// </summary>
public class ImagePublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    // Variables required for ROS communication
    public string cameraTopicName = "camera/color/image_raw/compressed";
    public string camInfoTopic = "/camera/color/camera_info";

    // Sensor
    public Camera imageCamera; 
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    public int qualityLevel = 50;
    
    // Message
    private CompressedImageMsg compressedImage;
    public string frameID = "camera";

    private Texture2D texture2D;
    private Rect rect;

    private uint seq;
    private float _timeStamp = 0f;

    public float stereo_offset = 0.01f;

    private CameraInfoMsg infoCamera;
    public bool is_stereo = false;

    private HeaderMsg header_msg;

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(cameraTopicName);
        ros.RegisterPublisher<CameraInfoMsg>(camInfoTopic);

        // Initialize renderer
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.ARGB32, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        RenderTexture renderTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24, 
                                                        RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        imageCamera.targetTexture = renderTexture;
        seq = 0;        
        // Initialize messages
        compressedImage = new CompressedImageMsg();
        compressedImage.header.frame_id = frameID;
        compressedImage.format = "jpeg";

        header_msg = new HeaderMsg();
        header_msg.frame_id = frameID;

        if (is_stereo){
            infoCamera = CameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, header_msg, stereo_offset);
        } else {
            infoCamera = CameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, header_msg);
        }
        // Call back
        Camera.onPostRender += UpdateImage;
        
    }

    private void UpdateImage(Camera cameraObject)
    {
        
        if (texture2D != null && cameraObject == imageCamera)
        {
            // Update time
            this._timeStamp = Time.time;

            uint sec = (uint)Math.Truncate(this._timeStamp);
            uint nanosec = (uint)( (this._timeStamp - sec)*1e+9 );
            
            var timeMessage = new TimeMsg(sec, nanosec);
            
            header_msg.seq = seq;
            header_msg.stamp.sec  = sec;
            header_msg.stamp.nanosec  = nanosec;

            
            //            compressedImage.header.Update();
            texture2D.ReadPixels(rect, 0, 0);
            compressedImage.data = texture2D.EncodeToJPG(qualityLevel);
            ros.Publish(cameraTopicName, compressedImage);
            ros.Publish(camInfoTopic, infoCamera);
            seq++;
        }
    }
}
