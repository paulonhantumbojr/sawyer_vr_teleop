using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

namespace Unity.Robotics
{
    /* Abstract class that subscribes to a set topic and receives the respective messages from that topic (The abstraction will make the script invisible in Unity's inspector) */
    public abstract class DataSubscriber<Msg> : MonoBehaviour where Msg: ROSTCPConnector.MessageGeneration.Message
    {
        /* Declare the camera topic where messages will be collected from. Possible topics to pick from the D435i RealSense camera:
         * "/joint_states"
         */
        [SerializeField] protected string _topic = ""; 

        // Declare the buffer queue of the message
        [SerializeField] protected int _queueSize = 1;

        // Declare the queue of incoming messages
        protected Queue<Msg> _incomingMessages;

        // Declare the latest messages received
        Msg _latestMessage;

        // Declare a flag for the new messages
        protected bool _newMessageAvailable = false;

        // ROS Connector
        ROSConnection m_Ros;

        // Start the necessary components to establish a ROS connection with the desired image messages
        protected virtual void Start()
        {
            // Create a Queue of _queueSize initial capacity (which can increases)
            _incomingMessages = new Queue<Msg>(_queueSize);
            
            // Establish the ROS connection
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.Subscribe<Msg>(_topic, ReceiveCallback);
        }

        // Function that continuosly runs as the 
        protected virtual void Update()
        {
            // If no new messages, do not return anything
            if(_incomingMessages.Count == 0)
                return;

            // Flush old messages if more than _queueSize image message/s is/are sent (image messages are sent continuosly so they can overload the buffer)
            while(_incomingMessages.Count > _queueSize)
            {
                _incomingMessages.Dequeue();
            }
        }

        // Return the flag signalling for every new message received
        protected bool NewMessageAvailable()
        {
            return _newMessageAvailable;
        }

        // Callback to receive respective messages
        protected void ReceiveCallback(Msg message)
        {    
            // If the buffer of messages has data (messages), load them into the cue and flag their receival
            if(message != null)
            {
                _latestMessage = message;
                _incomingMessages.Enqueue(_latestMessage);
                _newMessageAvailable = true;
            }
        }

        // Return the received message from the selected ROS topic
        protected Msg GetLatestMessage()
        {
            _newMessageAvailable = false;
            return _latestMessage;
        }

        // Return the queue of messages received from the selected ROS topic
        protected Queue<Msg> GetMessageQueue()
        {
            _newMessageAvailable = false;
            return _incomingMessages;
        }
    }
}