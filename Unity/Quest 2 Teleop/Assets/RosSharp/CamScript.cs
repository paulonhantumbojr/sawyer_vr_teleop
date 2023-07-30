

using UnityEngine;
using System.Collections.Generic;
using RosMessageTypes.Sensor;

namespace Unity.Robotics
{
    // Structure for texture functionality
    public struct Texture2DStamped
    {
        public float time;
        public Texture2D tex;
    }

    // Structure for image data
    public struct FrameDataStamped
    {
        public float time;
        public byte[] imageData;
    }

    /* Class that obtains compressed image messages and renders them as textures in Unity's skybox */
    public class CamScript : DataSubscriber<CompressedImageMsg>
    {
        private CompressedImageMsg _msg;
        private Texture2D _texture2D;
        private Queue<Texture2DStamped> _texture2DStampedQueue;
        private Texture2DStamped _latestTexture2DStamped;
        private byte[] _imageData;
        private float _frameTime;
        private FrameDataStamped _latestFrameDataStamped;
        private bool _ready = false;

        private Queue<byte[]> _frameDataQueue = new Queue<byte[]>();
        private const int _maxFrameQueueSize = 300; 
        private readonly object _frameQueueLock = new object();

        protected override void Start()
        {
            base.Start();

            // Declare the texture with the respective width and height, and queue
            _texture2D = new Texture2D(1, 1);
            _texture2DStampedQueue = new Queue<Texture2DStamped>(_queueSize);
        }

        protected override void Update()
        {
            base.Update();
            if(NewMessageAvailable())
            {
                _ready = false;
                _msg = GetLatestMessage();
                _imageData = _msg.data;
                _frameTime = (float)_msg.header.stamp.sec + (float)_msg.header.stamp.nanosec/1000000000;
                EnqueueFrame(_imageData);

                ImageConversion.LoadImage(_texture2D, _imageData);
                _latestTexture2DStamped.time = _frameTime;
                _latestTexture2DStamped.tex = _texture2D;
                _texture2DStampedQueue.Enqueue(_latestTexture2DStamped);
                if(_texture2DStampedQueue.Count > _queueSize) _texture2DStampedQueue.Dequeue();

                _latestFrameDataStamped.time = _frameTime;
                _latestFrameDataStamped.imageData = _imageData;
                _ready = true;
            }
        }

        public void EnqueueFrame(byte[] frameData)
        {
            lock (_frameQueueLock)
            {
                if (_frameDataQueue.Count < _maxFrameQueueSize)
                {
                    _frameDataQueue.Enqueue(frameData);
                    //Debug.Log($"Current Queue size: {_frameDataQueue.Count}");
                }
                else
                {
                    //Debug.LogWarning("Frame queue is full. Discarding frame.");
                }
            }
        }

        public byte[] DequeueFrame()
        {
            lock (_frameQueueLock)
            {
                if (_frameDataQueue.Count > 0)
                {
                    return _frameDataQueue.Dequeue();
                }
            }
            return null;
        }

        public Queue<byte[]> GetFrameQueue() 
        {
            lock (_frameQueueLock)
            {
                return _frameDataQueue;
            }
        }

        public int GetFrameQueueCount()
        {
            return _frameDataQueue.Count;
        }

        public bool isReady()
        {
            return _ready;
        }

        public Texture2D GetLatestTexture2D()
        {
            return _texture2D;
        }

        public Texture2DStamped GetLatestTexture2DStamped()
        {
            return _latestTexture2DStamped;
        }

        public Queue<Texture2DStamped> GetTexture2DStampedQueue()
        {
            return _texture2DStampedQueue;
        }

        public byte[] GetLatestFrameData()
        {
            return _imageData;
        }

        public FrameDataStamped GetLatestFrameDataStamped()
        {
            return _latestFrameDataStamped;
        }

    }
}