#ifndef SDFr_PROCEDURAL
#define SDFr_PROCEDURAL

// NOTE if using HDRP, LWRP or Core, more extensive functionality is found in:
// Core RP Library/ShaderLibrary/Common.hlsl

// Render Pipeline Core copyright © 2018 Unity Technologies ApS
// Licensed under the Unity Companion License for Unity-dependent projects--see [Unity Companion License](http://www.unity3d.com/legal/licenses/Unity_Companion_License). 

// draw procedural with 2 triangles has index order (0,1,2)  (0,2,3)
// 0 - 0,0
// 1 - 0,1
// 2 - 1,1
// 3 - 1,0
float2 GetQuadTexCoord(uint vertexID)
{
    uint topBit = vertexID >> 1;
    uint botBit = (vertexID & 1);
    float u = topBit;
    float v = (topBit + botBit) & 1; // produces 0 for indices 0,3 and 1 for 1,2
#if UNITY_UV_STARTS_AT_TOP
    v = 1.0 - v;
#endif
    return float2(u, v);
}

// 0 - 0,1
// 1 - 0,0
// 2 - 1,0
// 3 - 1,1
float4 GetQuadVertexPosition(uint vertexID, float z = UNITY_NEAR_CLIP_VALUE)
{
    uint topBit = vertexID >> 1;
    uint botBit = (vertexID & 1);
    float x = topBit;
    float y = 1 - (topBit + botBit) & 1; // produces 1 for indices 0,3 and 0 for 1,2
    return float4(x, y, z, 1.0);
}

uniform float4 _BlitScaleBias;
uniform float4 _BlitScaleBiasRt;

struct appdata_proc
{
    uint vertexID : SV_VertexID;
};

struct Varyings_Proc
{
    float2 texcoord : TEXCOORD0;
    float4 positionCS : SV_POSITION;
};

Varyings_Proc vert_proc_quad(appdata_proc input)
{
    Varyings_Proc output;
    output.positionCS = GetQuadVertexPosition(input.vertexID) * float4(_BlitScaleBiasRt.x, _BlitScaleBiasRt.y, 1, 1) + float4(_BlitScaleBiasRt.z, _BlitScaleBiasRt.w, 0, 0);
    output.positionCS.xy = output.positionCS.xy * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f); //convert to -1..1
    output.texcoord = GetQuadTexCoord(input.vertexID) * _BlitScaleBias.xy + _BlitScaleBias.zw;
    return output;   
}

#endif