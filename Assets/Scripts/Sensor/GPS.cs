using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace FRJ.Sensor
{
    public class GPS : MonoBehaviour
    {
        [SerializeField] private double _baseLatitude = 35.71020206575301;      // ��n�ǂ̈ܓx [m]
        [SerializeField] private double _baseLongitude = 139.81070039691542;    // ��n�ǂ̌o�x [m]
        [SerializeField] private double _baseAltitude = 3.0;                    // ��n�ǂ̕W���i�C�������j[m]
        [SerializeField] private int    _satelliteNum = 8;                      // �g�p�q����
        [SerializeField] private double _HDOP = 1.0;                            // �������x�ቺ��
        [SerializeField] private double _geoidHeight = 36.7071;                 // �W�I�C�h�� [m]
        [SerializeField] private float _updateRate = 10f; 

        private double _latitude;   // �ܓx [m]
        private double _longitude;  // �o�x [m]
        private double _altitude;   // �W�� [m]
        // private string _gprmc;      // GPRMC message
        private string _gpgga;      // GPGGA message
        // private string _gpvtg;      // GPVTG message
        // private string _gphdt;      // GPHDT message

        public float updateRate{ get => this._updateRate; }
        // public string gprmc { get => this._gprmc; }
        public string gpgga { get => this._gpgga; }
        // public string gpvtg { get => this._gpvtg; }
        // public string gphdt { get => this._gphdt; }

        private GeoCoordinate _gc;
        private NMEASerializer _serializer;

        public void Init()
        {
            this._gc = new GeoCoordinate(this._baseLatitude, this._baseLongitude);
            this._serializer = new NMEASerializer();
            this._serializer.geoidLevel = (float)this._geoidHeight;
            this._serializer.satelliteNum = this._satelliteNum;
            this._serializer.hdop = (float)this._HDOP;
        }

        public void updateGPS()
        {
            (this._latitude, this._longitude) = this._gc.XZ2LatLon(this.transform.position.x, this.transform.position.z);
            this._altitude = this._baseAltitude + this.transform.position.y;

            this._serializer.latitude = (float)this._latitude;
            this._serializer.longitude = (float)this._longitude;
            this._serializer.altitude = (float)this._altitude;
            
            this._gpgga = this._serializer.GPGGA();
        }
    }
}
