using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;

public static class TransformExtensions
{

    public static TransformMsg ToROSTransform(this Transform tfUnity)
    {
        return new TransformMsg(
            // Using vector/quaternion To<>() because Transform.To<>() doesn't use localPosition/localRotation
            tfUnity.localPosition.To<FLU>(),
            tfUnity.localRotation.To<FLU>());
    }

    public static TransformStampedMsg ToROSTransformStamped(this Transform tfUnity, double timeStamp, string frame_id)
    {
        return new TransformStampedMsg(
            new HeaderMsg{
            frame_id= tfUnity.parent.gameObject.name,
            stamp = new TimeMsg
                {
                    sec = (uint)timeStamp,
                    nanosec = (uint)((timeStamp - Math.Floor(timeStamp)) * Clock.k_NanoSecondsInSeconds)
                }
            },
            // new HeaderMsg(new TimeStamp(timeStamp), tfUnity.parent.gameObject.name),
            tfUnity.gameObject.name,
            tfUnity.ToROSTransform());
    }
}
        //   new HeaderMsg{
        //     stamp = new TimeMsg
        //         {
        //             sec = timeStamp.Seconds,
        //             nanosec = timeStamp.NanoSeconds,
        //         }, 
        //     frame_id = tfUnity.parent.gameObject.name
        //     },