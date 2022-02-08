using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;


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

        private double rl_wheel_vel;
        private double rr_wheel_vel;
        private double fl_wheel_vel;
        private double fr_wheel_vel;
    

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
            ros.Subscribe<JointStateMsg>("joint_states", ReceiveROSCmd);
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

        void ReceiveROSCmd(JointStateMsg joint_state_msg){
            Debug.Log("ReceiveROSCmd");
            rl_wheel_vel = joint_state_msg.velocity[2];
            rr_wheel_vel = joint_state_msg.velocity[3];
            fl_wheel_vel = joint_state_msg.velocity[0];
            fr_wheel_vel = joint_state_msg.velocity[1];

            SetSpeed(rl_wheel, (float) rl_wheel_vel);
            SetSpeed(rr_wheel, (float) rr_wheel_vel);
            SetSpeed(fl_wheel, (float) fl_wheel_vel);
            SetSpeed(fr_wheel, (float) fr_wheel_vel);
                          
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
                drive.targetVelocity = wheelSpeed*Mathf.Rad2Deg;
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
              
        // Update is called once per frame
        void Update()
        {
            
        }
    }
}