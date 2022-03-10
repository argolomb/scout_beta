using System.Collections;

using UnityEngine;

public class WheelRotation : MonoBehaviour{

    public WheelCollider wheelcollider;

    public Transform wheelTransform;    
    
    void Start() {
        wheelcollider.GetWorldPose(out Vector3 _position, out Quaternion _rotation);
    }


    void FixedUpdate(){
        wheelcollider.GetWorldPose(out Vector3 _position, out Quaternion _rotation);
        wheelTransform.position = _position;
        wheelTransform.rotation = _rotation*Quaternion.Euler(0,270,0);
        
    }


}