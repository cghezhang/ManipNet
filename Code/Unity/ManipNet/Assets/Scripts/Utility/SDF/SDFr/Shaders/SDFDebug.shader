Shader "XRA/SDFr"
{
	Properties
	{
	}
	
    HLSLINCLUDE

    #pragma target 4.5
    #pragma only_renderers d3d11 ps4 xboxone vulkan metal switch
    #pragma enable_d3d11_debug_symbols
    #pragma multi_compile_instancing
    
    //remove if using com.unity.render-pipelines.core
    #include "UnityCG.cginc"
    
    #include "SDFrProcedural.hlsl"
    #include "SDFrVolumeTex.hlsl"
	#include "SDFrUtilities.hlsl"

    uniform float4x4 _PixelCoordToViewDirWS;
    
    Texture2D _BlueNoiseRGBA;
    SamplerState sampler_point_repeat_BlueNoiseRGBA;
    float4 _BlueNoiseRGBA_TexelSize; 
    
    float4 screenBlueNoise( float2 uv )
    {	
        //keep 0-1
        float2 shift = frac(_Time.xy);
        //then scale by with and height of the target texture
        shift *= _ScreenParams.xy;
        //then round it down (removes fractional part)
        shift = floor(shift);
        //how many times the texture fits into screen if 1:1
        float2 screenByTex = (_ScreenParams.xy/_BlueNoiseRGBA_TexelSize.zw);
        float2 screenCoords = (uv + shift) * (screenByTex);
        return _BlueNoiseRGBA.SampleLevel( sampler_point_repeat_BlueNoiseRGBA, screenCoords, 0 );
    }
    
    ENDHLSL
	
	SubShader
    {
        Tags{ "RenderType" = "Opaque" }

        //render signed distance field with world normals (remapped to 0-1)
        Pass //0
        { 
            Name "SDFrVisualize"
            Tags { "LightMode" = "SDFrVisualize" }
            
            ZTest LEqual
            ZWrite on 
            Blend off
            Cull front
                        
            HLSLPROGRAM 
			// TODO: In 2019 - change to shader_feature_local 
			#pragma shader_feature _ SDFr_VISUALIZE_STEPS SDFr_VISUALIZE_HEATMAP SDFr_VISUALIZE_DIST

            #pragma vertex vert_proc_quad
            #pragma fragment Frag
		
            #define MAX_STEPS 1024
            #define EPSILON 0.003
            #define NORMAL_DELTA 0.03
            
            Texture3D _SDFVolumeTex; 
            float4x4 _SDFVolumeLocalToWorld;
            float4x4 _SDFVolumeWorldToLocal; 
            float3 _SDFVolumeExtents; //xyz extents of volume
            float _SDFVolumeFlip;
            float _SDFPreviewEpsilon = EPSILON;
            float _SDFPreviewNormalDelta = NORMAL_DELTA;

			inline float DistanceFunction(float3 rayPosLS)
			{
				float3 vp = rayPosLS + 0.5;

				// Testing filtering and mip levels
				float sample = _SDFVolumeTex.SampleLevel(sdfr_sampler_linear_clamp, vp, 0).r;
				// float sample = _SDFVolumeTex.SampleLevel(sdfr_sampler_trilinear_clamp, vp, 0).r;
				// float sample = _SDFVolumeTex.Sample(sdfr_sampler_trilinear_clamp, vp).r;
				// if (_SDFVolumeFlip < 0) sample = -sample;
				return (_SDFVolumeFlip < 0) ? -sample : sample;
			}

			// Should normalDeltas be cached?
			inline float3 GenerateNormalsFast(float normalDelta, float dist, float3 rayPosLS)
			{
				float3 nx = rayPosLS + float3(normalDelta, 0, 0);
				float3 ny = rayPosLS + float3(0, normalDelta, 0);
				float3 nz = rayPosLS + float3(0, 0, normalDelta);
				float dx = DistanceFunction(nx) - dist;
				float dy = DistanceFunction(ny) - dist;
				float dz = DistanceFunction(nz) - dist;
				return normalize(float3(dx, dy, dz));
			}

			inline float3 GenerateNormals(float normalDelta, float dist, float3 rayPosLS)
			{
				float dx = DistanceFunction(rayPosLS + float3(normalDelta, 0, 0)) - DistanceFunction(rayPosLS - float3(normalDelta, 0, 0));
				float dy = DistanceFunction(rayPosLS + float3(0, normalDelta, 0)) - DistanceFunction(rayPosLS - float3(0, normalDelta, 0));
				float dz = DistanceFunction(rayPosLS + float3(0, 0, normalDelta)) - DistanceFunction(rayPosLS - float3(0, 0, normalDelta));
				return normalize(float3(dx, dy, dz));
			}

            struct OutputPS
            {
                half4 color : COLOR0;
                float depth : DEPTH;
            };
            
            OutputPS Raymarch( float3 roWS, float3 rdWS, float3 reWS )
            {
                float normalDelta = max(0.0001,_SDFPreviewNormalDelta);
                float eps = max(0.0001,_SDFPreviewEpsilon);
            
                OutputPS o = (OutputPS)0;
                                
                //ray origin world to local space
                float3 roLS = mul(_SDFVolumeWorldToLocal,float4(roWS,1)).xyz;
                //ray end world to local
                float3 reLS = mul(_SDFVolumeWorldToLocal,float4(reWS,1)).xyz;
                //ray direction world to local
                float3 rdLS = normalize(reLS-roLS); //mul((float3x3)_SDFVolumeWorldToLocal,rdWS.xyz);
                
                //min and max of bounds
                float3 minAABB = -_SDFVolumeExtents.xyz;
                float3 maxAABB =  _SDFVolumeExtents.xyz;
                                
                //intersection value is 0 (ray origin) to 1 (ray end) 
                //x is enter intersection, y is exit intersection
                //NOTE! this intersection is only for axis-aligned bounding boxes
                //so any rays in world space must be taken into local space if the volume bounds are rotated
                //if the AABB never rotates then the intersection can be in world space & save matrix multiply
                float2 intersection = LineAABBIntersect( roLS, reLS, minAABB, maxAABB );
                float3 enterLS = lerp(roLS,reLS,saturate(intersection.x));
                float3 exitLS  = lerp(roLS,reLS,saturate(intersection.y));
                                
                float distanceToEnter = distance(roLS,enterLS);
                float distanceToExit  = distance(roLS,exitLS);
                
                //the distance traveled within volume
                float distanceInVolume = distanceToExit - distanceToEnter;
                             
                //if the ray is intersecting the bounds
                UNITY_BRANCH
                if ( intersection.x < intersection.y && intersection.x < 1 )
                {
                    //accumulate distance samples
                    float dist = 0; 
                    //accumulate the steps
                    int steps = 0;
                    
                    UNITY_LOOP
                    while( dist < distanceInVolume && steps < MAX_STEPS )
                    {
                        //current position of ray
                        //since it is in local space always use the intersection enter position
                        //the intersection is on surface of bounds or at camera in bounds
                        float3 rayPosLS = enterLS + rdLS * dist;
                        
                        //length(_SDFVolumeLocalToWorld[0])
                        
                        //the ray position at current step
                        //normalize into volume texture space 
                        rayPosLS /= _SDFVolumeExtents.xyz*2;
						float d = DistanceFunction(rayPosLS);
                        
                        UNITY_BRANCH
                        if ( d < eps )
                        {
						//	float3 normalLS = GenerateNormalsFast(normalDelta, d, rayPosLS);
							float3 normalLS = GenerateNormals(normalDelta * 0.5, d, rayPosLS);

                            //object to world space normals 
                            float3 normalWS = mul((float3x3)_SDFVolumeLocalToWorld,normalLS);
                            
                            //local to world ray hit position
                            float3 rayHitWS = mul(_SDFVolumeLocalToWorld,float4(rayPosLS,1)).xyz;
                            
							// BUG: Depth value appears incorrect in preview mode - it intersects with geometry when it shouldn't.
						
                            //NOTE only needed for depth if mixing with depth buffer
                            // float4 ndc = UnityObjectToClipPos(float4(rayHitWS, 1)); 
							float4 ndc = mul(UNITY_MATRIX_MVP,float4(rayHitWS,1));
						//	float4 ndc = mul(UNITY_MATRIX_VP, float4(rayHitWS, 1));
                            float realDepth = ndc.z/ndc.w;                   
                            o.depth = realDepth;
							
						//	float4 clippos = mul(UNITY_MATRIX_IT_MV, float4(rayHitWS, 1.0));
						//	o.depth = clippos.z;

#ifdef SDFr_VISUALIZE_DIST
							o.color = half4(0, 0, dist / 10.0, 1);
#elif SDFr_VISUALIZE_STEPS
							o.color = half4(steps / (float)MAX_STEPS, 0, 0, 1);
#elif SDFr_VISUALIZE_HEATMAP
							// HeatMap - Green = minimal, Red = maximum number of steps
							float	stepf = steps / (float)MAX_STEPS;
							float	hue = lerp(0.33, 0.0, stepf);
							float3	rgb = HsvToRgb(float3(hue, 1, 1));
							o.color = half4(rgb, 1);
#else
							//visualize world normals 
							o.color = half4(normalWS*0.5 + 0.5, 1);
#endif

                            return o;
                        }
                        dist += d;
						steps++;
                    }
                }
                //else did not intersect volume, discard 
                discard;
                return o;
            }
            
            OutputPS Frag( Varyings_Proc input )
            {            
                //ray origin
                float3 ro = _WorldSpaceCameraPos;
                //ray from camera to pixel coordinates in world space
                float3 rd = -normalize(mul(float3(input.positionCS.xy, 1.0), (float3x3)_PixelCoordToViewDirWS));
                
                //if using blue noise or similar to jitter rays
                //float blueNoise = screenBlueNoise( input.texcoord ).r;
                //ro += rd * blueNoise * 0.1;
                
                float3 re = ro + rd * _ProjectionParams.z;
                
                return Raymarch( ro, rd, re );
            }
            
            ENDHLSL
        }
	}
}