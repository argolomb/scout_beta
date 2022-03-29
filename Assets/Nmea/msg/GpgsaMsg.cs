//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Nmea
{
    [Serializable]
    public class GpgsaMsg : Message
    {
        public const string k_RosMessageName = "nmea_msgs/Gpgsa";
        public override string RosMessageName => k_RosMessageName;

        //  Message from GPGSA NMEA String
        public HeaderMsg header;
        public string message_id;
        public string auto_manual_mode;
        public byte fix_mode;
        public byte[] sv_ids;
        public float pdop;
        public float hdop;
        public float vdop;

        public GpgsaMsg()
        {
            this.header = new HeaderMsg();
            this.message_id = "";
            this.auto_manual_mode = "";
            this.fix_mode = 0;
            this.sv_ids = new byte[0];
            this.pdop = 0.0f;
            this.hdop = 0.0f;
            this.vdop = 0.0f;
        }

        public GpgsaMsg(HeaderMsg header, string message_id, string auto_manual_mode, byte fix_mode, byte[] sv_ids, float pdop, float hdop, float vdop)
        {
            this.header = header;
            this.message_id = message_id;
            this.auto_manual_mode = auto_manual_mode;
            this.fix_mode = fix_mode;
            this.sv_ids = sv_ids;
            this.pdop = pdop;
            this.hdop = hdop;
            this.vdop = vdop;
        }

        public static GpgsaMsg Deserialize(MessageDeserializer deserializer) => new GpgsaMsg(deserializer);

        private GpgsaMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.message_id);
            deserializer.Read(out this.auto_manual_mode);
            deserializer.Read(out this.fix_mode);
            deserializer.Read(out this.sv_ids, sizeof(byte), deserializer.ReadLength());
            deserializer.Read(out this.pdop);
            deserializer.Read(out this.hdop);
            deserializer.Read(out this.vdop);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.message_id);
            serializer.Write(this.auto_manual_mode);
            serializer.Write(this.fix_mode);
            serializer.WriteLength(this.sv_ids);
            serializer.Write(this.sv_ids);
            serializer.Write(this.pdop);
            serializer.Write(this.hdop);
            serializer.Write(this.vdop);
        }

        public override string ToString()
        {
            return "GpgsaMsg: " +
            "\nheader: " + header.ToString() +
            "\nmessage_id: " + message_id.ToString() +
            "\nauto_manual_mode: " + auto_manual_mode.ToString() +
            "\nfix_mode: " + fix_mode.ToString() +
            "\nsv_ids: " + System.String.Join(", ", sv_ids.ToList()) +
            "\npdop: " + pdop.ToString() +
            "\nhdop: " + hdop.ToString() +
            "\nvdop: " + vdop.ToString();
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