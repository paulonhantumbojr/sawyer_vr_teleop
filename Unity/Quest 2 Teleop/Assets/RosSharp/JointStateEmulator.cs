

using UnityEngine;
using System;
using RosMessageTypes.Sensor;

namespace Unity.Robotics
{
    /* Class that obtains compressed image messages and renders them as textures in Unity's skybox */
    public class JointStateEmulator : DataSubscriber<JointStateMsg>
    {
        [SerializeField] protected GameObject m_joint;
        [SerializeField] protected int _jointIndex;

        // Declare Sawyer's DH Parameters (ref: http://mfg.rethinkrobotics.com/wiki/Robot_Hardware#tab=Sawyer)
        // Key: [d, a, alpha, offset]
        // d - translation in z-axis (y-axis in Unity)
        // a - translation in the x-axis (z-axis in Unity)
        // alpha - rotation about x-axis (z-axis in Unity)
        // offset - displacement along the z-axis (y-axis in Unity)

        double[,] dhParameters = new double[7, 4]
        {
            { 0.317, 0.081, -((Math.PI / 2) * Mathf.Rad2Deg), 0 },                           // J0 
            { 0.1925, 0, ((Math.PI / 2) * Mathf.Rad2Deg), ((Math.PI / 2) * Mathf.Rad2Deg) }, // J1 
            { 0.4, 0, -((Math.PI / 2) * Mathf.Rad2Deg), 0 },                                 // J2
            { -0.1685, 0, ((Math.PI / 2) * Mathf.Rad2Deg), 0 },                              // J3 
            { 0.4, 0, -((Math.PI / 2) * Mathf.Rad2Deg), 0 },                                 // J4
            { 0.1363, 0, ((Math.PI / 2) * Mathf.Rad2Deg), 0 },                               // J5
            { 0.13375, 0, 0, -((Math.PI / 2) * Mathf.Rad2Deg) }                              // J6
        };

        protected override void Update()
        {
            // Calling 'base.Update()' ensures that the base class's update logic is executed alongside any additional logic you've implemented in your derived classes
            base.Update();

            // Update the joint positions of the URDF whenever a new message is received
            if (NewMessageAvailable())
            {
                // Obtain the latest joint message and correspondingly update the joint positions of the robot
                JointStateMsg jointState = GetLatestMessage();
                UpdateSingleJoint(jointState);
            }
        }

        // Functionality to update the robot's joint positions
        private void UpdateSingleJoint(JointStateMsg jointState)
        {
            // Set the pose of the finger joints when the state is published separately from the default Sawyer states
            if (jointState.name.Length == 1)
            {
                return;
            } 

                // Map the position of the specified joint index to match the simulated one
                double jointPosition = jointState.position[_jointIndex];

                // Set the transform of the joint based on the /joint_states message
                float jointPositionDegrees = (float)(jointPosition * Mathf.Rad2Deg);

                // Set the joint rotation based on the rotation frame of the scene to match the local rotation to Unity's axes
                // For setting the rotations, refer to the 'alpha' and 'offset' parameters within the DH Parameter table
                switch (_jointIndex)
                {
                    /* Set the cases based on the order (index) of the joint states */
                    // Joint 0  
                    case 1: // NOTE: Change to 1 if subscribing to /robot/joint_states (using the real Sawyer)
                        m_joint.transform.localRotation = Quaternion.Euler(0f, -jointPositionDegrees, 0f);
                        break;
                    // Joint 1 
                    case 2:
                        m_joint.transform.localRotation = Quaternion.Euler((jointPositionDegrees + (float)dhParameters[1, 3]), 0f, (float)dhParameters[1, 2]);
                        break;
                    // Joint 2 
                    case 3:
                        m_joint.transform.localRotation = Quaternion.Euler(-jointPositionDegrees, 0f, (float)dhParameters[2, 2]);
                        break;
                    // Joint 3 
                    case 4:
                        m_joint.transform.localRotation = Quaternion.Euler(jointPositionDegrees, 0f, (float)dhParameters[3, 2]);
                        break;
                    // Joint 4 
                    case 5:
                        m_joint.transform.localRotation = Quaternion.Euler(-jointPositionDegrees, 0f, (float)dhParameters[4, 2]);
                        break;
                    // Joint 5
                    case 6:
                        m_joint.transform.localRotation = Quaternion.Euler(jointPositionDegrees, 0f, (float)dhParameters[5, 2]);
                        break;
                    // Joint 6                                                                                            
                    case 7:
                        m_joint.transform.localRotation = Quaternion.Euler(-(((8 * Mathf.PI / 9) * Mathf.Rad2Deg) + jointPositionDegrees), 0f, (float)dhParameters[6, 3]);
                        break;
                    default:
                        Debug.LogWarning("Invalid Joint Index");
                        break;
            }
        }
    }
}