
/* Class to visualise Rviz markers in Unity */

using UnityEngine;
using RosMessageTypes.Visualization;

namespace Unity.Robotics
{
    public class VizMarkers : DataSubscriber<MarkerArrayMsg>
    {
        [SerializeField] protected GameObject m_baseOrigin; // Reference to the base GameObject of the robot in the scene for the (0, 0, 0) position
        [SerializeField] protected float m_markerLifetime;  // Lifetime of the markers (in seconds)

        protected override void Update()
        {
            // Calling 'base.Update()' ensures that the base class's update logic is executed alongside any additional logic you've implemented in your derived classes
            base.Update();

            // Update the marker position whenever a new message is received
            if (NewMessageAvailable())
            {
                // Obtain the latest joint message and correspondingly update the joint positions of the robot
                MarkerArrayMsg markerarray = GetLatestMessage();
                UpdateMarker(markerarray);
            }
        }

        // Functionality to update the marker positions within Unity's scene
        private void UpdateMarker(MarkerArrayMsg mrk)
        {
            // Logic to display marker array in Unity's scene goes here
            foreach (var marker in mrk.markers)
            {
                // Here, we handle the case for spherical markers. For other types, you'll need to add different cases.
                if (marker.type == 0)
                {
                    // Create a new spherical GameObject to represent the marker
                    var markerObject = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                    markerObject.name = "Goal";

                    // Calculate and set the marker position relative to the base GameObject
                    Vector3 markerPosition = m_baseOrigin.transform.TransformPoint(new Vector3(-(float)marker.pose.position.y, (float)marker.pose.position.z, (float)marker.pose.position.x));
                    markerObject.transform.position = markerPosition;

                    // Similarly, set the orientation of the marker relative to the base GameObject
                    Quaternion markerOrientation = m_baseOrigin.transform.rotation * new Quaternion(-(float)marker.pose.orientation.y, (float)marker.pose.orientation.z, (float)marker.pose.orientation.x, (float)marker.pose.orientation.w);
                    markerObject.transform.rotation = markerOrientation;

                    // Set the scale of the marker
                    markerObject.transform.localScale = new Vector3(
                        0.1f,
                        0.1f,
                        0.1f
                    );

                    // Set the gradient material of the marker
                    Material markerMaterial = Resources.Load<Material>("MARKER"); // Extract it from the 'Resources' folder
                    markerObject.GetComponent<Renderer>().material = markerMaterial;

                    // Destroy the marker object after the specified lifetime
                    Destroy(markerObject, m_markerLifetime);
                }
            }
        }
    }
}
