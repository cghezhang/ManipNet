#ifndef XRA_RAYMARCH_TEX3D
#define XRA_RAYMARCH_TEX3D

SamplerState sdfr_sampler_linear_clamp;
SamplerState sdfr_sampler_trilinear_clamp;

struct SDFrVolumeData
{
    float4x4 WorldToLocal;
    float3 Extents;
};




inline float2 LineAABBIntersect(float3 ro, float3 re, float3 aabbMin, float3 aabbMax)
{
    float3 invRd = 1.0 / (re - ro);
    float3 mini = invRd * (aabbMin - ro);
    float3 maxi = invRd * (aabbMax - ro);
    float3 closest = min(mini, maxi);
    float3 furthest = max(mini, maxi);
    
    float2 intersections;
    //furthest near intersection
    intersections.x = max(closest.x, max(closest.y, closest.z));
    //closest far intersection
    intersections.y = min(furthest.x, min(furthest.y, furthest.z));
    //map 0 ray origin, 1 ray end
    return saturate(intersections);
}

//incoming ray is world space
inline float DistanceFunctionTex3D( in float3 rayPosWS, in float3 rayOrigin, in float3 rayEnd, in SDFrVolumeData data, in Texture3D tex )
{
    float4x4 w2l = data.WorldToLocal;
    float3 extents = data.Extents;
    
    //take ray to local space of bounds (now the bounds is axis-aligned relative to the ray)
    float3 originLocal = mul(w2l,float4(rayOrigin,1)).xyz;
    float3 endLocal = mul(w2l,float4(rayEnd,1)).xyz;
     
    float2 intersection = LineAABBIntersect(originLocal, endLocal, -extents, extents);
    
    //if the ray intersects 
    if ( intersection.x < intersection.y && intersection.x < 1 )
    {    
        float3 rayPosLocal = mul(w2l,float4(rayPosWS,1)).xyz;
        rayPosLocal /= extents.xyz*2;
        rayPosLocal += 0.5; //texture space
        //values are -1 to 1
        float sample = tex.SampleLevel( sdfr_sampler_linear_clamp, rayPosLocal, 0 ).r;
		sample *= length(extents); //scale by magnitude of bound extent -- NC: Should this by times 2 as extents is halfsize?
        return sample;
    }
    //TODO not really consistent 
    return distance(rayOrigin,rayEnd);
}

inline float DistanceFunctionTex3DFast(in float3 rayPosWS, in SDFrVolumeData data, in Texture3D tex)
{
	float4x4 w2l = data.WorldToLocal;
	float3 extents = data.Extents;

	float3 rayPosLocal = mul(w2l, float4(rayPosWS, 1)).xyz;
	rayPosLocal /= extents.xyz * 2;
	rayPosLocal += 0.5; //texture space
	//values are -1 to 1
	float sample = tex.SampleLevel(sdfr_sampler_linear_clamp, rayPosLocal, 0).r;
	sample *= length(extents); //scale by magnitude of bound extent
	return sample;
}


// Find the min distance to AABB
inline float DistanceToAABB(in float3 rayOrigin, in float3 rayEnd, in SDFrVolumeData data)
{
	float4x4	w2l = data.WorldToLocal;
	float3		extents = data.Extents;

	//take ray to local space of bounds (now the bounds is axis-aligned relative to the ray)
	float3 originLocal = mul(w2l, float4(rayOrigin, 1)).xyz;
	float3 endLocal = mul(w2l, float4(rayEnd, 1)).xyz;

	float2 intersection = LineAABBIntersect(originLocal, endLocal, -extents, extents);

	if (intersection.x < intersection.y && intersection.x < 1)
	{
		return intersection.x * distance(rayOrigin, rayEnd);
	}
	return distance(rayOrigin, rayEnd);
}
#endif