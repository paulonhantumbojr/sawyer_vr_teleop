

using UnityEngine;
using System;
using RosMessageTypes.Sensor;
using TMPro;

namespace Unity.Robotics
{
    /* Class that obtains compressed image messages and renders them as textures in Unity's skybox */
    public class JointPositionControl : DataSubscriber<JointStateMsg>
    {
        [SerializeField] protected TextMeshPro m_LimitState;
        [SerializeField] protected int m_Index;

        // Sawyer Joint Limits
        float[,] jointLimits = new float[7, 2]
        {
            { -175, +175 },  // J0 
            { -219, +131 },  // J1 
            { -175, +175 },  // J2
            { -175, +175 },  // J3 
            { -175, +175 },  // J4
            { -175, +175 },  // J5
            { -270, +270 }   // J6
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
                UpdateLimitState(jointState);
            }
        }

        // Functionality to update the robot's joint positions
        private void UpdateLimitState(JointStateMsg jointState)
        {
            // Set the pose of the finger joints when the state is published separately from the default Sawyer states
            if (jointState.name.Length == 1)
            {
                return;
            }

            // Map the position of the specified joint index to match the simulated one
            double jointPosition = jointState.position[m_Index];

            // Set the transform of the joint based on the /joint_states message
            float jointPositionDegrees = (float)(jointPosition * Mathf.Rad2Deg);
            float _tolerance = 2;

            // Set the joint rotation based on the rotation frame of the scene to match the local rotation to Unity's axes
            // For setting the rotations, refer to the 'alpha' and 'offset' parameters within the DH Parameter table
            switch (m_Index)
            {
                /* Set the cases based on the order (index) of the joint states */
                // Joint 0  
                case 1: // NOTE: Change to 1 if subscribing to /robot/joint_states (using the real Sawyer)
                    if (jointPositionDegrees >= (jointLimits[0, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[0, 0] + _tolerance) )
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    } else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                // Joint 1 
                case 2:
                    if (jointPositionDegrees >= (jointLimits[1, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[1, 0] + _tolerance))
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    }
                    else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                // Joint 2 
                case 3:
                    if (jointPositionDegrees >= (jointLimits[2, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[2, 0] + _tolerance))
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    }
                    else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                // Joint 3 
                case 4:
                    if (jointPositionDegrees >= (jointLimits[3, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[3, 0] + _tolerance))
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    }
                    else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                // Joint 4 
                case 5:
                    if (jointPositionDegrees >= (jointLimits[4, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[4, 0] + _tolerance))
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    }
                    else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                // Joint 5
                case 6:
                    if (jointPositionDegrees >= (jointLimits[5, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[5, 0] + _tolerance))
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    }
                    else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                // Joint 6                                                                                            
                case 7:
                    if (jointPositionDegrees >= (jointLimits[6, 1] - _tolerance) || jointPositionDegrees <= (jointLimits[6, 0] + _tolerance))
                    {
                        m_LimitState.text = "At Limit";
                        m_LimitState.color = Color.red;
                    }
                    else
                    {
                        m_LimitState.text = "Within Limit";
                        m_LimitState.color = Color.green;
                    }
                    break;
                default:
                    Debug.LogWarning("Invalid Limits");
                    break;
            }
        }
    }
}