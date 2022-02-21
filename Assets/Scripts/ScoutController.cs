using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS};
    public class ScoutController : MonoBehaviour
    {
        ROSConnection ros;
        /*
        public GameObject rear_left_wheel;
        public GameObject rear_right_wheel;
        public GameObject front_left_wheel;
        public GameObject front_right_wheel;
        */
        public Vector3 tensor;
        public Rigidbody rb;
        public WheelCollider rear_left_wheel_colider;
        public WheelCollider rear_right_wheel_colider;
        public WheelCollider front_left_wheel_colider;
        public WheelCollider front_right_wheel_colider;
        public float maxLinearSpeed = 1.0f; //  m/s
        public float maxAcceleration = 3.0f; //
        public float maxRotationalSpeed = 2.0f;// rad/s
        public float maxAngularAceleration = 6.0f;
        public float wheelRadius = 0.16459f; //meters
        public float trackWidth = 0.58306f; // meters Distance between tyres     

        private float torqueMax = 12f;

        private double rl_wheel_vel;
        private double rr_wheel_vel;
        private double fl_wheel_vel;
        private double fr_wheel_vel;
        private float rosLinearVel = 0f;
        private float rosAngularVel = 0f;

        // Start is called before the first frame update
        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
            rb.inertiaTensor = tensor;
            Debug.Log("Start");
        }

        void ReceiveROSCmd(TwistMsg cmd_vel){
            rosLinearVel = (float) cmd_vel.linear.x;
            rosAngularVel = (float) cmd_vel.angular.z;
                          
        }

        [Tooltip("Proportional constant (counters current error)")]
	    public float Kp_rl = 20f;
	
	    [Tooltip("Integral constant (counters cumulated error)")]
	    public float Ki_rl = 1.2f;
	
	    [Tooltip("Derivative constant (fights oscillation)")]
	    public float Kd_rl = 0.1f;

        private float integral_rl;

        private float lastError_rl;

        void rear_left_wheel_torque(float dt){

            float left_speed = rosLinearVel - (rosAngularVel*trackWidth)/2;

            left_speed = (float) System.Math.Round(left_speed,2);

            float rear_left_speed = (float) (2*System.Math.PI*wheelRadius*(rear_left_wheel_colider.rpm))/60;

            rear_left_speed =(float) System.Math.Round(rear_left_speed,2);

            float left_speed_error = left_speed - rear_left_speed;

            float derivative = (left_speed_error-lastError_rl)/dt;
            integral_rl+= left_speed_error*dt;
            lastError_rl = left_speed_error;

            float value_left = (float) System.Math.Round(Kp_rl*left_speed_error + Ki_rl*integral_rl + Kd_rl*derivative, 2);

            value_left = torqueclip(value_left);

            if(left_speed > 0){
                rear_left_wheel_colider.brakeTorque = 0; 
                rear_left_wheel_colider.motorTorque = value_left;
            } 
            else if (left_speed < 0){
                rear_left_wheel_colider.brakeTorque = 0; 
                rear_left_wheel_colider.motorTorque = value_left;
            } 
            else if (left_speed == 0){
                rear_left_wheel_colider.motorTorque = 0;
                rear_left_wheel_colider.brakeTorque = torqueMax;
                integral_rl = 0; 

            }
            Debug.Log("RL Speed: " + left_speed + "\n" +
                    "Current RL Speed: "+ rear_left_speed +"\n" +
                    "RL Speed Error :"+left_speed_error +"\n" +
                    "RL torque: "+ value_left);
        }

        [Tooltip("Proportional constant (counters current error)")]
	    public float Kp_fl = 20f;
	
	    [Tooltip("Integral constant (counters cumulated error)")]
	    public float Ki_fl = 1.2f;
	
	    [Tooltip("Derivative constant (fights oscillation)")]
	    public float Kd_fl = 0.1f;

        private float integral_fl;

        private float lastError_fl;
        void front_left_wheel_torque(float dt){

            float left_speed = rosLinearVel - (rosAngularVel*trackWidth)/2;

            left_speed = (float) System.Math.Round(left_speed,2);

            float front_left_speed = (float) (2*System.Math.PI*wheelRadius*(front_left_wheel_colider.rpm))/60;

            front_left_speed = (float) System.Math.Round(front_left_speed,2);

            float left_speed_error = left_speed - front_left_speed;

            float derivative = (left_speed_error-lastError_fl)/dt;
            integral_fl+= left_speed_error*dt;
            lastError_fl = left_speed_error;

            float value_left = (float) System.Math.Round(Kp_fl*left_speed_error + Ki_fl*integral_fl + Kd_fl*derivative, 2);

            value_left = torqueclip(value_left);

            if(left_speed > 0){
                front_left_wheel_colider.brakeTorque = 0;
                front_left_wheel_colider.motorTorque = value_left;
            } 
            else if (left_speed < 0){
                front_left_wheel_colider.brakeTorque = 0;
                front_left_wheel_colider.motorTorque = value_left;
            } 
            else if (left_speed == 0){
                front_left_wheel_colider.motorTorque = 0;
                front_left_wheel_colider.brakeTorque = torqueMax;
                integral_fl=0;
            }
            Debug.Log("FL Speed: " + left_speed + "\n" +
                    "Current FL Speed: "+ front_left_speed +"\n" +
                    "FL Speed Error :"+left_speed_error +"\n" +
                    "FL torque: "+ value_left);
        }

        [Tooltip("Proportional constant (counters current error)")]
	    public float Kp_fr = 20f;
	
	    [Tooltip("Integral constant (counters cumulated error)")]
	    public float Ki_fr = 1.2f;
	
	    [Tooltip("Derivative constant (fights oscillation)")]
	    public float Kd_fr = 0.1f;

        private float integral_fr;

        private float lastError_fr;
        void front_right_wheel_torque(float dt){

            float right_speed = rosLinearVel + (rosAngularVel*trackWidth)/2;
            right_speed = (float) System.Math.Round(right_speed,2);

            float front_right_speed = (float) (2*System.Math.PI*wheelRadius*(front_right_wheel_colider.rpm))/60;

            front_right_speed = (float) System.Math.Round(front_right_speed,2);

            float right_speed_error = right_speed - front_right_speed;

            float derivative = (right_speed_error-lastError_fr)/dt;
            integral_fr+= right_speed_error*dt;
            lastError_fr = right_speed_error;

            float value_right = (float) System.Math.Round(Kp_fr*right_speed_error + Ki_fr*integral_fr + Kd_fr*derivative,2);

            value_right = torqueclip(value_right);

            if(right_speed > 0){
                front_right_wheel_colider.brakeTorque = 0;
                front_right_wheel_colider.motorTorque = value_right;
            } 
            else if (right_speed < 0){
                front_right_wheel_colider.brakeTorque = 0;
                front_right_wheel_colider.motorTorque = value_right;
            } 
            else if (right_speed == 0){
                front_right_wheel_colider.motorTorque = 0;
                front_right_wheel_colider.brakeTorque = torqueMax;
                integral_fr = 0;

            }
            Debug.Log("FR Speed: " + right_speed +"\n" +
                    "Current FR Speed: "+ front_right_speed +"\n" +  
                    "FR Speed Error :"+right_speed_error +"\n" +
                    "FR torque: "+ value_right);

            
        }

        [Tooltip("Proportional constant (counters current error)")]
	    public float Kp_rr = 20f;
	
	    [Tooltip("Integral constant (counters cumulated error)")]
	    public float Ki_rr = 1.2f;
	
	    [Tooltip("Derivative constant (fights oscillation)")]
	    public float Kd_rr = 0.1f;

        private float integral_rr;

        private float lastError_rr;
        void rear_right_wheel_torque(float dt){

            float right_speed = rosLinearVel + (rosAngularVel*trackWidth)/2;
            right_speed = (float) System.Math.Round(right_speed,2);

            float rear_right_speed = (float) (2*System.Math.PI*wheelRadius*(rear_right_wheel_colider.rpm))/60;

            rear_right_speed = (float) System.Math.Round(rear_right_speed,2);

            float right_speed_error = right_speed - rear_right_speed;

            float derivative = (right_speed_error-lastError_rr)/dt;
            integral_rr+= right_speed_error*dt;
            lastError_rr = right_speed_error;

            float value_right = (float) System.Math.Round(Kp_rr*right_speed_error + Ki_rr*integral_rr + Kd_rr*derivative,2);

            value_right = torqueclip(value_right);

            if(right_speed > 0){
                rear_right_wheel_colider.brakeTorque = 0; 
                rear_right_wheel_colider.motorTorque = value_right;
            } 
            else if (right_speed < 0){
                rear_right_wheel_colider.brakeTorque = 0; 
                rear_right_wheel_colider.motorTorque = value_right;
            } 
            else if (right_speed == 0){
                rear_right_wheel_colider.motorTorque = 0;
                rear_right_wheel_colider.brakeTorque = torqueMax;
                integral_rr = 0; 

            }
            Debug.Log("RR Speed: " + right_speed +"\n" +
                    "Current RR Speed: "+ rear_right_speed +"\n" +  
                    "RR Speed Error :"+right_speed_error +"\n" +
                    "RR torque: "+ value_right);

            
        }

        float torqueclip(float value){
            if (value > torqueMax){
                return torqueMax;
            } else if (value < -torqueMax){
                return -torqueMax; 
            } else if(value == 0){
                value = 0;
            }
            return value;
            }


        // Update is called once per frame
        void FixedUpdate()
        {
            //torque_controller(rosLinearVel, rosAngularVel, Time.deltaTime);
            rear_left_wheel_torque(Time.deltaTime);
            rear_right_wheel_torque(Time.deltaTime);
            front_right_wheel_torque(Time.deltaTime);
            front_left_wheel_torque(Time.deltaTime);

        }
    }
}