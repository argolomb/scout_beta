using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;


namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS};
    public class ScoutController : MonoBehaviour
    {
        ROSConnection ros;
        public GameObject rear_left_wheel;
        public GameObject rear_right_wheel;
        public GameObject front_left_wheel;
        public GameObject front_right_wheel;

        private ArticulationBody rl_wheel;
        private ArticulationBody rr_wheel;
        private ArticulationBody fl_wheel;
        private ArticulationBody fr_wheel;

        private float rosLinear = 0f;
        private float rosAngular = 0f;
        public float maxLinearSpeed = 1.5f; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.16459f; //meters
        public float trackWidth = 0.58306f; // meters Distance between tyres
        public float forceLimit = 100f;
        public float damping = 1f;
        // Start is called before the first frame update
        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
            rl_wheel = rear_left_wheel.GetComponent<ArticulationBody>();
            rr_wheel = rear_right_wheel.GetComponent<ArticulationBody>();
            fl_wheel = front_left_wheel.GetComponent<ArticulationBody>();
            fr_wheel = front_right_wheel.GetComponent<ArticulationBody>();
            
            
            SetParameters(rl_wheel);
            SetParameters(rr_wheel);
            SetParameters(fl_wheel);
            SetParameters(fr_wheel);
            Debug.Log("Start");
        }

        void ReceiveROSCmd(TwistMsg cmdVel){
            Debug.Log("ReceiveROSCmd");
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            RobotInput(rosLinear, rosAngular);                   
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }
        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float fl_rotSpeed = (speed / wheelRadius);
            float fr_rotSpeed= fl_rotSpeed;
            float rl_rotSpeed = (speed / wheelRadius);
            float rr_rotSpeed = rl_rotSpeed;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            Debug.Log(wheelSpeedDiff);

            if (rotSpeed != 0 && speed == 0)
            {
                fl_rotSpeed = (fl_rotSpeed + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                fr_rotSpeed = -(fr_rotSpeed + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                rl_rotSpeed = (rl_rotSpeed + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                rr_rotSpeed = -(rr_rotSpeed + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                fl_rotSpeed *= Mathf.Rad2Deg*-1;
                fr_rotSpeed *= Mathf.Rad2Deg;
                rl_rotSpeed *= Mathf.Rad2Deg*-1;
                rr_rotSpeed *= Mathf.Rad2Deg;
            }
            SetSpeed(rl_wheel, rl_rotSpeed);
            SetSpeed(rr_wheel, rr_rotSpeed);
            SetSpeed(fl_wheel, fl_rotSpeed);
            SetSpeed(fr_wheel, fr_rotSpeed);
        }
        // Update is called once per frame
        void Update()
        {
            
        }
    }
}