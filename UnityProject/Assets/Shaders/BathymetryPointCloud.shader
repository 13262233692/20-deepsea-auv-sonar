Shader "DeepseaAUV/BathymetryPointCloud"
{
    Properties
    {
        _PointSize ("Point Size", Float) = 0.25
        _Opacity   ("Global Opacity", Range(0,1)) = 1.0
        _MinDepth  ("Min Depth (m)", Float) = 10
        _MaxDepth  ("Max Depth (m)", Float) = 200
        [Toggle] _UseVertexColor ("Use Vertex Color", Float) = 1
    }
    SubShader
    {
        Tags { "Queue"="Transparent+5" "RenderType"="Transparent" "IgnoreProjector"="True" }
        LOD 200

        Pass
        {
            Name "POINT_CLOUD"
            Tags { "LightMode"="ForwardBase" }
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off
            Offset 0, -1

            CGPROGRAM
            #pragma target 5.0
            #pragma vertex   vert
            #pragma geometry geom
            #pragma fragment frag
            #pragma multi_compile_fwdbase nolightmap nodynlightmap nodirlightmap novertexlights

            #include "UnityCG.cginc"
            #include "Lighting.cginc"

            struct Vin
            {
                float4 pos : POSITION;
                float4 col : COLOR;
                float2 uv  : TEXCOORD0;
            };

            struct V2G
            {
                float4 pos : SV_POSITION;
                float4 col : COLOR;
                float  size : TEXCOORD0;
                float3 worldPos : TEXCOORD1;
            };

            struct G2F
            {
                float4 pos : SV_POSITION;
                float4 col : COLOR;
                float2 uv : TEXCOORD0;
                float3 worldPos : TEXCOORD1;
                float  psize : TEXCOORD2;
            };

            float _PointSize;
            float _Opacity;
            float _MinDepth;
            float _MaxDepth;
            float _UseVertexColor;

            float3 bathymetryRamp(float t)
            {
                t = saturate(t);
                float3 c0 = float3(0.10, 0.05, 0.35);
                float3 c1 = float3(0.00, 0.35, 0.70);
                float3 c2 = float3(0.00, 0.75, 0.65);
                float3 c3 = float3(0.95, 0.85, 0.20);
                float3 c4 = float3(0.95, 0.30, 0.10);
                float3 r;
                if (t < 0.25) r = lerp(c0, c1, t / 0.25);
                else if (t < 0.5) r = lerp(c1, c2, (t - 0.25) / 0.25);
                else if (t < 0.75) r = lerp(c2, c3, (t - 0.5) / 0.25);
                else r = lerp(c3, c4, (t - 0.75) / 0.25);
                return r;
            }

            V2G vert(Vin v)
            {
                V2G o;
                o.worldPos = mul(unity_ObjectToWorld, v.pos).xyz;
                o.pos = UnityObjectToClipPos(v.pos);
                float d = length(_WorldSpaceCameraPos - o.worldPos);
                float ps = _PointSize * (300.0 / max(1.0, d));
                o.size = max(1.0, ps);
                if (v.uv.x > 0.5) o.size = max(1.0, v.uv.x);

                if (_UseVertexColor > 0.5)
                {
                    o.col = v.col;
                }
                else
                {
                    float depth = max(0, -o.worldPos.y);
                    float t = saturate((depth - _MinDepth) / max(1e-4, _MaxDepth - _MinDepth));
                    o.col = float4(bathymetryRamp(t), _Opacity);
                }
                o.col.a *= _Opacity;
                return o;
            }

            [maxvertexcount(4)]
            void geom(point V2G input[1], inout TriangleStream<G2F> stream)
            {
                if (input[0].col.a < 0.005) return;

                V2G v = input[0];
                float2 dirs[4] = {
                    float2(-1, -1),
                    float2( 1, -1),
                    float2(-1,  1),
                    float2( 1,  1)
                };
                float2 uvs[4] = {
                    float2(0, 0),
                    float2(1, 0),
                    float2(0, 1),
                    float2(1, 1)
                };

                float s = v.size * 0.5;
                [unroll]
                for (int i = 0; i < 4; i++)
                {
                    G2F o;
                    o.pos = v.pos;
                    o.pos.xy += dirs[i] * s;
                    o.col = v.col;
                    o.uv = uvs[i];
                    o.worldPos = v.worldPos;
                    o.psize = v.size;
                    stream.Append(o);
                }
                stream.RestartStrip();
            }

            float4 frag(G2F i) : SV_Target
            {
                float2 d = i.uv - 0.5;
                float r2 = dot(d, d) * 4.0;
                float a = smoothstep(1.0, 0.0, r2);
                if (a < 0.05) discard;
                float4 c = i.col;
                c.a *= a;
                return c;
            }
            ENDCG
        }
    }
}
