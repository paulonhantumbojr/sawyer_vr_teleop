Shader "Unlit/VerticalGradient"
{
    Properties
    {
        _TopColour ("Top Gradient Colour", Color) = (1, 1, 1, 1)
        _BottomColour ("Bottom Gradient Colour", Color) = (0, 0, 0, 1)
        _BottomGradientSpace ("Bottom Gradient Space", Range(0, 1)) = 0.5
    }

    SubShader
    {
        Tags { "RenderType" = "Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            fixed4 _TopColour;
            fixed4 _BottomColour;
            float _BottomGradientSpace;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv * float2(1, _BottomGradientSpace);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                return lerp(_BottomColour, _TopColour, i.uv.y);
            }

            ENDCG
        }
    }
}
