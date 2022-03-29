//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Nmea
{
    [Serializable]
    public class GpgsvSatelliteMsg : Message
    {
        public const string k_RosMessageName = "nmea_msgs/GpgsvSatellite";
        public override string RosMessageName => k_RosMessageName;

        //  Satellite data structure used in GPGSV messages
        //  PRN number of the satellite
        //  GPS = 1..32
        //  SBAS = 33..64
        //  GLO = 65..96
        public byte prn;
        //  Elevation, degrees. Maximum 90
        public byte elevation;
        //  Azimuth, True North degrees. [0, 359]
        public ushort azimuth;
        //  Signal to noise ratio, 0-99 dB. -1 when null in NMEA sentence (not tracking)
        public sbyte snr;

        public GpgsvSatelliteMsg()
        {
            this.prn = 0;
            this.elevation = 0;
            this.azimuth = 0;
            this.snr = 0;
        }

        public GpgsvSatelliteMsg(byte prn, byte elevation, ushort azimuth, sbyte snr)
        {
            this.prn = prn;
            this.elevation = elevation;
            this.azimuth = azimuth;
            this.snr = snr;
        }

        public static GpgsvSatelliteMsg Deserialize(MessageDeserializer deserializer) => new GpgsvSatelliteMsg(deserializer);

        private GpgsvSatelliteMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.prn);
            deserializer.Read(out this.elevation);
            deserializer.Read(out this.azimuth);
            deserializer.Read(out this.snr);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.prn);
            serializer.Write(this.elevation);
            serializer.Write(this.azimuth);
            serializer.Write(this.snr);
        }

        public override string ToString()
        {
            return "GpgsvSatelliteMsg: " +
            "\nprn: " + prn.ToString() +
            "\nelevation: " + elevation.ToString() +
            "\nazimuth: " + azimuth.ToString() +
            "\nsnr: " + snr.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
