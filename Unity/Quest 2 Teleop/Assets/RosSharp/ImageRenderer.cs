using UnityEngine;
using UnityEngine.UI;

namespace Unity.Robotics
{
    /* Class that renders image into a chosen plane in the designed scene */
    public class ImageRenderer : MonoBehaviour
    {
        [SerializeReference] private RenderTexture renderTexture;
        [SerializeReference] private CamScript imageSub;
        [SerializeField] private float displayFrameRate = 30f; // Set the frame rate for displaying images (30 for RGB images, up to 90 for depth images)
        private Texture2D displayTexture2D; // Declare the variable for displaying the textures
        private float m_lastFrameUpdateTime = 0f;

        private void Start()
        {
            displayTexture2D = new Texture2D(1, 1);
        }

        // Display the desired textures once the compressed image messages have been received
        private void Update()
        {
            // If the received image is ready for displaying
            if(imageSub.isReady())
            {
                displayTexture2D = imageSub.GetLatestTexture2D();
                if(Time.time - m_lastFrameUpdateTime > 1/displayFrameRate)
                {
                    m_lastFrameUpdateTime = Time.time;

                    // Graphics.Blit uses the dedicated graphics card in your computer to do this instead of using integrated graphics (your cpu) 
                    Graphics.Blit(displayTexture2D, renderTexture);
                }
            }
        }

        public void SetDisplayFrameRate(float framerate)
        {
            displayFrameRate = framerate;
        }

        public void SetDisplayFrameRate(Text framerate)
        {
            displayFrameRate = float.Parse(framerate.text);
        }

        // Functionality to clear the rendered textures
        public void ClearRenderTexture()
        {
            RenderTexture oldActive = RenderTexture.active;
            RenderTexture.active = renderTexture;

            GL.Clear(true, true, Color.clear);

            RenderTexture.active = oldActive;
        }
    }
}