/* Class that obtains the headset data and publishes them as ROS messages */

using UnityEngine;
using UnityEngine.XR;
using TMPro;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

[RequireComponent(typeof(InitialiseVR))]
public class PublishVRData : MonoBehaviour
{
    // Initialise attributes
    [SerializeField] public TextMeshPro m_GripperStateText;
    private InitialiseVR _initialiseVR;
    ROSConnection m_Ros;

    // Declare the ROS headset topics (These topics are generated automatically since they don't inherently exist)
    [SerializeField] string m_HeadsetPoseTopic = "/oculus_quest2/headset/pose";
    [SerializeField] string m_LeftControllerPoseTopic = "/oculus_quest2/left/pose";
    [SerializeField] string m_RightControllerPoseTopic = "/oculus_quest2/right/pose";
    [SerializeField] string m_LeftControllerGripButtonStateTopic = "/oculus_quest2/left/grip_state";
    [SerializeField] string m_RightControllerGripButtonStateTopic = "/oculus_quest2/right/grip_state";
    [SerializeField] string m_LeftControllerTriggerButtonStateTopic = "/oculus_quest2/left/trigger_state";
    [SerializeField] string m_RightControllerTriggerButtonStateTopic = "/oculus_quest2/right/trigger_state";
    [SerializeField] string m_PrimaryButtonStateTopic = "/oculus_quest2/primary/button_state";
    [SerializeField] string m_SecondaryButtonStateTopic = "/oculus_quest2/secondary/button_state";
    [SerializeField] string m_LeftPrimary2DAxisButtonStateTopic = "/oculus_quest2/left/primary2d/button_state";
    [SerializeField] string m_RightPrimary2DAxisButtonStateTopic = "/oculus_quest2/right/primary2d/button_state";

    // On start, call the InitialiseVR object to initialise all of the headset components (remove the 'private' definition)
    void Start()
    {
        // Establish the ROS connection instances for the new topics
        m_Ros = ROSConnection.GetOrCreateInstance();

        // Declare and create the new publishing topics for the extraction of the desired VR data
        m_Ros.RegisterPublisher<PoseMsg>(m_HeadsetPoseTopic);
        m_Ros.RegisterPublisher<PoseMsg>(m_LeftControllerPoseTopic);
        m_Ros.RegisterPublisher<PoseMsg>(m_RightControllerPoseTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_LeftControllerGripButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_RightControllerGripButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_LeftControllerTriggerButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_RightControllerTriggerButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_PrimaryButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_SecondaryButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_LeftPrimary2DAxisButtonStateTopic);
        m_Ros.RegisterPublisher<BoolMsg>(m_RightPrimary2DAxisButtonStateTopic);

        // Initialise the VR Object
        _initialiseVR = GetComponent<InitialiseVR>();
    }

    // Function that publishes the device data as ROS messages. Called once per frame (if changed to a separate name, then a UI button will act as a trigger for publishing the VR messages)
    void Update()
    {
        // Check if the vrText reference is null or not assigned
        if (!m_GripperStateText)
        {
            // Try to find the Text component on the GameObject automatically
            m_GripperStateText = GetComponent<TextMeshPro>();

            // If still null, log a warning and return
            if (!m_GripperStateText)
            {
                Debug.LogWarning("Text component not found on the VRTextController's GameObject. Make sure a Text component is attached or assigned.");
                return;
            }
        }

        // Publishing Headset Pose
        if (_initialiseVR._HMD.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 headsetPosition) && 
            _initialiseVR._HMD.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion headsetRotation))
        {
            // Assign the headset position to its message (_headsetPosition)
            PoseMsg headsetPoseMsg = new PoseMsg
            {
                /* Going from Unity world space to ROS world space requires a conversion. Unity's coordinate space has x Right, y Up, and z Forward 
                 * (hence "RUF" coordinates); ROS has x Forward, y Left and z Up (hence "FLU"). So a Unity (x,y,z) coordinate is equivalent to the 
                 * ROS (z,-x,y) coordinate. These conversions are done by the To<FLU> function in the ROS-TCP-Connector package's */
                position = headsetPosition.To<FLU>(),
                orientation = headsetRotation.To<FLU>()
            };

            // Publish the pose information to the /oculus_quest2/headset
            m_Ros.Publish(m_HeadsetPoseTopic, headsetPoseMsg);
        }

        // Left Controller position
        if (_initialiseVR._leftController.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 leftcontrollerPosition) && 
            _initialiseVR._leftController.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion leftcontrollerRotation))
        {
            // Assign the left controller position to its message (_leftcontrollerPosition)
            PoseMsg leftcontrollerPoseMsg = new PoseMsg
            {
                position = leftcontrollerPosition.To<FLU>(),
                orientation = leftcontrollerRotation.To<FLU>()
            };

            // Publish the pose information to the /oculus_quest2/left/pose
            m_Ros.Publish(m_LeftControllerPoseTopic, leftcontrollerPoseMsg);
        }

        // Left Controller position
        if (_initialiseVR._rightController.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 rightcontrollerPosition) &&
            _initialiseVR._rightController.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rightcontrollerRotation))
        {
            // Assign the right controller position to its message (_rightcontrollerPosition)
            PoseMsg rightcontrollerPoseMsg = new PoseMsg
            {
                position = rightcontrollerPosition.To<FLU>(),
                orientation = rightcontrollerRotation.To<FLU>()
            };

            // Publish the pose information to the /oculus_quest2/right/pose
            m_Ros.Publish(m_RightControllerPoseTopic, rightcontrollerPoseMsg);
        }

        // Left Grip Button state
        if (_initialiseVR._leftController.TryGetFeatureValue(CommonUsages.gripButton, out bool leftGripButtonState))
        {
            BoolMsg leftGripButtonStateMsg = new BoolMsg
            {
                data = leftGripButtonState
            };

            // Publish the grip button state to the /oculus_quest2/left/grip_state
            m_Ros.Publish(m_LeftControllerGripButtonStateTopic, leftGripButtonStateMsg);
        }

        // Right Grip Button state
        if (_initialiseVR._rightController.TryGetFeatureValue(CommonUsages.gripButton, out bool rightGripButtonState))
        {
            BoolMsg rightGripButtonStateMsg = new BoolMsg
            {
                data = rightGripButtonState
            };

            // Publish the grip button state to the /oculus_quest2/right/grip_state
            m_Ros.Publish(m_RightControllerGripButtonStateTopic, rightGripButtonStateMsg);
        }

        // Left Trigger Button State
        if (_initialiseVR._leftController.TryGetFeatureValue(CommonUsages.triggerButton, out bool leftTriggerButtonState))
        {
            BoolMsg leftTriggerButtonStateMsg = new BoolMsg
            {
                data = leftTriggerButtonState
            };

            // Publish the trigger button state to the /oculus_quest2/left/trigger_state
            m_Ros.Publish(m_LeftControllerTriggerButtonStateTopic, leftTriggerButtonStateMsg);
        }

        // Right Trigger Button state
        if (_initialiseVR._rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool rightTriggerButtonState))
        {
            BoolMsg rightTriggerButtonStateMsg = new BoolMsg
            {
                data = rightTriggerButtonState
            };

            // Publish the trigger button state to the /oculus_quest2/right/trigger_state
            m_Ros.Publish(m_RightControllerTriggerButtonStateTopic, rightTriggerButtonStateMsg);
        }

        // Primary Button press state
        if (_initialiseVR._rightController.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryButtonState))
        {
            BoolMsg primaryButtonStateMsg = new BoolMsg
            {
                data = primaryButtonState
            };

            // Publish the trigger button state to the /oculus_quest2/primary/button_state
            m_Ros.Publish(m_PrimaryButtonStateTopic, primaryButtonStateMsg);

            // Change the text within the Unity environment to match the action of the gripper
            if (!primaryButtonState)
            {
                m_GripperStateText.text = "Not Engaged";
                m_GripperStateText.color = Color.red;
            }
            else
            {
                m_GripperStateText.text = "Engaged";
                m_GripperStateText.color = Color.green;
            }
        }

        // Secondary Button press state
        if (_initialiseVR._rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryButtonState))
        {
            BoolMsg secondaryButtonStateMsg = new BoolMsg
            {
                data = secondaryButtonState
            };

            // Publish the trigger button state to the /oculus_quest2/secondary/button_state
            m_Ros.Publish(m_SecondaryButtonStateTopic, secondaryButtonStateMsg);

            // Change the text within the Unity environment to match the action of the gripper
            if (secondaryButtonState)
            {
                m_GripperStateText.text = "Calibrating";
                m_GripperStateText.color = Color.yellow;
            }
        }

        // Left Primary 2D Axis Button press state
        if (_initialiseVR._leftController.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool primary2dButtonStateL))
        {
            BoolMsg leftPrimary2dButtonStateMsg = new BoolMsg
            {
                data = primary2dButtonStateL
            };

            // Publish the trigger button state to the /oculus_quest2/left/primary2d/button_state
            m_Ros.Publish(m_LeftPrimary2DAxisButtonStateTopic, leftPrimary2dButtonStateMsg);
        }

        // Right Primary 2D Axis Button press state
        if (_initialiseVR._rightController.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool primary2dButtonStateR))
        {
            BoolMsg rightPrimary2dButtonStateMsg = new BoolMsg
            {
                data = primary2dButtonStateR
            };

            // Publish the trigger button state to the /oculus_quest2/right/primary2d/button_state
            m_Ros.Publish(m_RightPrimary2DAxisButtonStateTopic, rightPrimary2dButtonStateMsg);
        }
    }
}