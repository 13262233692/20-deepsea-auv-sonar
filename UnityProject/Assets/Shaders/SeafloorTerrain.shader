Shader "DeepseaAUV/SeafloorTerrain"
{
    Properties
    {
        _BaseColor  ("Base Color", Color) = (0.05, 0.08, 0.12, 1)
        _SlopeColor ("Slope Color", Color) = (0.08, 0.05, 0.03, 1)
        _DepthRim   ("Depth Rim Color", Color) = (0.0, 0.3, 0.5, 1)
        _Glossiness ("Smoothness", Range(0,1)) = 0.1
        _Metallic   ("Metallic", Range(0,1)) = 0.0
        _NoiseScale ("Noise Scale", Float) = 5.0
        _NoiseAmount("Noise Amount", Range(0,1)) = 0.3
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        LOD 200

        Pass
        {
            Name "FORWARD"
            Tags { "LightMode"="ForwardBase" }
            Cull Back
            ZWrite On

            CGPROGRAM
            #pragma target 4.5
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_fwdbase
            #pragma multi_compile_fog

            #include "UnityCG.cginc"
            #include "Lighting.cginc"

            struct Vin
            {
                float4 pos : POSITION;
                float3 nrm : NORMAL;
                float2 uv  : TEXCOORD0;
            };

            struct V2F
            {
                float4 pos   : SV_POSITION;
                float3 worldPos : TEXCOORD0;
                float3 worldNrm : TEXCOORD1;
                float2 uv    : TEXCOORD2;
                float  depth : TEXCOORD3;
                UNITY_FOG_COORDS(4)
            };

            float4 _BaseColor;
            float4 _SlopeColor;
            float4 _DepthRim;
            float  _Glossiness;
            float  _Metallic;
            float  _NoiseScale;
            float  _NoiseAmount;

            float hash(float2 p)
            {
                return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453);
            }
            float noise(float2 p)
            {
                float2 i = floor(p);
                float2 f = frac(p);
                float a = hash(i);
                float b = hash(i + float2(1,0));
                float c = hash(i + float2(0,1));
                float d = hash(i + float2(1,1));
                float2 u = f*f*(3.0-2.0*f);
                return lerp(a,b,u.x) + (c-a)*u.y*(1.0-u.x) + (d-b)*u.x*u.y;
            }
            float fbm(float2 p)
            {
                float v = 0, a = 0.5;
                for (int i=0; i<4; i++) { v += a * noise(p); p *= 2.02; a *= 0.5; }
                return v;
            }

            V2F vert(Vin v)
            {
                V2F o;
                o.pos = UnityObjectToClipPos(v.pos);
                o.worldPos = mul(unity_ObjectToWorld, v.pos).xyz;
                o.worldNrm = UnityObjectToWorldNormal(v.nrm);
                o.uv  = v.uv;
                o.depth = -o.worldPos.y;
                UNITY_TRANSFER_FOG(o, o.pos);
                return o;
            }

            float4 frag(V2F i) : SV_Target
            {
                float3 N = normalize(i.worldNrm);
                float slope = 1.0 - saturate(N.y);
                float2 wp = i.worldPos.xz * _NoiseScale;
                float n = fbm(wp) * 0.5 + fbm(wp * 3.7) * 0.3 + fbm(wp * 11.3) * 0.2;
                n = n * 2.0 - 1.0;

                float3 base = lerp(_BaseColor.rgb, _SlopeColor.rgb, smoothstep(0.0, 0.8, slope));
                base = lerp(base, base + float3(0.05, 0.03, -0.02), n * _NoiseAmount);

                float depthFade = smoothstep(0, 300, i.depth);
                base = lerp(base, base * 0.2, depthFade * 0.7);

                float rim = pow(saturate(1.0 - abs(N.y)), 3.0);
                base = lerp(base, _DepthRim.rgb, rim * 0.25 * (1.0 - depthFade));

                // simple lighting with one dim directional + ambient
                float3 L = normalize(_WorldSpaceLightPos0.xyz);
                float ndl = max(0.0, dot(N, L));
                float3 light = _LightColor0.rgb * ndl;
                float3 ambient = ShadeSH9(float4(N, 1.0));
                float3 col = base * (ambient * 0.9 + light * 1.2);

                col = pow(abs(col), 0.9); // slightly desaturate deep sea
                UNITY_APPLY_FOG(i.fogCoord, col);
                return float4(col, 1.0);
            }
            ENDCG
        }
    }
    FallBack "Standard"
}
