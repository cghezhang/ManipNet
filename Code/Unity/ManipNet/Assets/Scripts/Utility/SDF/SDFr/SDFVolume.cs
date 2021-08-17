//NOTE: Burst + Unity.Mathematics is ~2x faster
//if not using comment this out
#define USE_BURST_AND_MATH

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;


#if USE_BURST_AND_MATH
using Unity.Mathematics;
using Unity.Burst;
// Aliases
using Random	= Unity.Mathematics.Random;
using vec3		= Unity.Mathematics.float3;
using vec4		= Unity.Mathematics.float4;
#else
// Aliases
using Random	= UnityEngine.Random;
using vec3		= UnityEngine.Vector3;
using vec4		= UnityEngine.Vector4;
#endif

namespace SDFr
{
    public class SDFVolume : AVolume<SDFVolume>
    {
        #if USE_BURST_AND_MATH
        private const string strProgressTitle = "SDFr [Burst]";
        #else
        private const string strProgressTitle = "SDFr";
        #endif
        private const string strProgress = "Generating signed distance field...";
        
        private const int LAYER_FRONT_GEO = 4; //"water" builtin
        private int LAYER_MASK_FRONT = 1 << LAYER_FRONT_GEO;
        private const int LAYER_BACK_GEO = 5; //"UI" builtin
        private int LAYER_MASK_BACK = 1 << LAYER_BACK_GEO;
        
        private Action<SDFVolume,float[],float,object> onBakeComplete;
        
        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);

            if (!disposing) return;
            onBakeComplete = null;
        }
        
        public void Bake( int raySamples, List<Renderer> renderers, Action<SDFVolume,float[],float,object> bakeComplete, object passthrough = null )
        {
            onBakeComplete = bakeComplete;

            int progressInterval = _settings.CellCount / 4;
            
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            vec3 halfVoxel = _settings.HalfVoxel;

            //adjusted to best timings from testing but it could vary by CPU
            int calcRayLengthBatchCount = 32;
            calcRayLengthBatchCount = Mathf.Clamp(calcRayLengthBatchCount,1,raySamples);
            int raycastBatchCount = 8;
            raycastBatchCount = Mathf.Clamp(raycastBatchCount,1,raySamples);
            int prepareRaysBatchCount = 64;
            prepareRaysBatchCount = Mathf.Clamp(prepareRaysBatchCount,1,raySamples);
            int compareBatchCount = 128;
            
            //for raycast method front facing geo and flipped backfacing geo is required
            List<Collider> geoFront = new List<Collider>();
            List<Collider> geoBack = new List<Collider>();
            CreateColliders( ref renderers, ref geoFront, ref geoBack );
            
            //prepare data
            NativeArray<float> distances = new NativeArray<float>(_settings.CellCount, Allocator.TempJob);
            NativeArray<CellResults> allResults = new NativeArray<CellResults>(_settings.CellCount,Allocator.TempJob);
            
            //constant for all cells
            NativeArray<vec3> sphereSamples = new NativeArray<vec3>(raySamples, Allocator.TempJob);
            //NativeArray<vec3> randomDirections = new NativeArray<vec3>(raySamples, Allocator.TempJob);
            NativeArray<vec4> volumePlanes = new NativeArray<vec4>(6, Allocator.TempJob);
            
            GetUniformPointsOnSphereNormalized(ref sphereSamples);
            //GetRandomDirections( _halfVoxel*settings.JitterScale, settings.JitterSeed, ref randomDirections);
            
            vec3 aabbMin = BoundsWorldAABB.min;
            vec3 aabbMax = BoundsWorldAABB.max;            
            //the max ray length, used to normalize all resulting distances
            //so they are treated as 0 to 1 within a volume
            float aabbMagnitude = BoundsWorldAABB.size.magnitude;
            
            Plane pl = new Plane(Vector3.right,aabbMin);
            Plane pr = new Plane(Vector3.left,aabbMax);
            Plane pd = new Plane(Vector3.up,aabbMin);
            Plane pu = new Plane(Vector3.down,aabbMax);
            Plane pb = new Plane(Vector3.forward,aabbMin); 
            Plane pf = new Plane(Vector3.back,aabbMax);

			volumePlanes[0] = new vec4( pl.normal.x, pl.normal.y, pl.normal.z, pl.distance);
            volumePlanes[1] = new vec4( pr.normal.x, pr.normal.y, pr.normal.z, pr.distance);
            volumePlanes[2] = new vec4( pd.normal.x, pd.normal.y, pd.normal.z, pd.distance);
            volumePlanes[3] = new vec4( pu.normal.x, pu.normal.y, pu.normal.z, pu.distance);
            volumePlanes[4] = new vec4( pb.normal.x, pb.normal.y, pb.normal.z, pb.distance);
            volumePlanes[5] = new vec4( pf.normal.x, pf.normal.y, pf.normal.z, pf.distance);

            //iterate each cell performing raycasted samples
            for (int i = 0; i < _settings.CellCount; i++)
            {
#if UNITY_EDITOR
				if (i % progressInterval == 0)
                {
                    EditorUtility.DisplayProgressBar(strProgressTitle,strProgress,i/(float)_settings.CellCount);
                }
#endif

				vec3 positionWS = _settings.ToPositionWS(i,LocalToWorldNoScale);
                vec3 centerVoxelWS = positionWS + halfVoxel;
                
                NativeArray<float> rayLengths = new NativeArray<float>(raySamples, Allocator.TempJob);
                NativeArray<RaycastCommand> allRaycastsFront = new NativeArray<RaycastCommand>(raySamples, Allocator.TempJob);
                NativeArray<RaycastCommand> allRaycastsBack = new NativeArray<RaycastCommand>(raySamples, Allocator.TempJob);
                NativeArray<RaycastHit> frontHits = new NativeArray<RaycastHit>(raySamples, Allocator.TempJob);
                NativeArray<RaycastHit> backHits = new NativeArray<RaycastHit>(raySamples, Allocator.TempJob);
                
                //calculate the ray lengths, just so rays are clipped within the volume when raycasting
                CalculateRayLengths calcRayLengths = new CalculateRayLengths
                {
                    Samples = sphereSamples,
                    VolumePlanes = volumePlanes,
                    RayLengths = rayLengths,
                    RayLength = aabbMagnitude,
                    RayOrigin = centerVoxelWS
                };
                JobHandle rayLengthHandle = calcRayLengths.Schedule(raySamples,calcRayLengthBatchCount);
                
                //prepare raycasts front
                PrepareRaycastCommands frontPrc = new PrepareRaycastCommands
                {
                    Samples = sphereSamples,
                    RayLengths = rayLengths,
                    LayerMask = LAYER_MASK_FRONT,
                    Raycasts = allRaycastsFront,
                    RayOrigin = centerVoxelWS,
                };
                //prepare raycasts back
                PrepareRaycastCommands backPrc = new PrepareRaycastCommands
                {
                    Samples = sphereSamples,
                    RayLengths = rayLengths,
                    LayerMask = LAYER_MASK_BACK,
                    Raycasts = allRaycastsBack,
                    RayOrigin = centerVoxelWS,
                };

                //schedule front raycasts
                JobHandle prepareFrontHandle = frontPrc.Schedule(
                    raySamples, prepareRaysBatchCount, rayLengthHandle);
                JobHandle scheduleFrontHandle = RaycastCommand.ScheduleBatch(
                    allRaycastsFront, frontHits, raycastBatchCount, prepareFrontHandle);
                
                //schedule back raycasts
                JobHandle prepareBackHandle = backPrc.Schedule(
                    raySamples, prepareRaysBatchCount, rayLengthHandle);
                JobHandle scheduleBackHandle = RaycastCommand.ScheduleBatch(
                    allRaycastsBack, backHits, raycastBatchCount, prepareBackHandle);
                
                //combine handles
                JobHandle frontBackHandle = JobHandle.CombineDependencies(scheduleFrontHandle, scheduleBackHandle);
                
                //process results and put into current cell index
                ProcessHits processHits = new ProcessHits
                {
                    FrontHits = frontHits,
                    BackHits = backHits,
                    Results = allResults.Slice(i,1),
                };

                JobHandle cellHandle = processHits.Schedule(frontBackHandle);   
                cellHandle.Complete();
                
                rayLengths.Dispose();
                allRaycastsFront.Dispose();
                allRaycastsBack.Dispose();
                frontHits.Dispose();
                backHits.Dispose();
                
            } //for each cell
            
            //final distances
            CompareDistances compareDistances = new CompareDistances
            {
                Distances = distances,
                Results = allResults
            };
            
            JobHandle compareDistancesHandle = compareDistances.Schedule(_settings.CellCount,compareBatchCount);
            compareDistancesHandle.Complete();
            
            stopwatch.Stop();
            Debug.Log("SDF bake completed in "+stopwatch.Elapsed.ToString("mm\\:ss\\.ff"));
#if UNITY_EDITOR
			EditorUtility.ClearProgressBar();
#endif
			float[] distancesOut = new float[_settings.CellCount];
            distances.CopyTo(distancesOut);
            
            //cleanup all the temp arrays
            distances.Dispose();
            allResults.Dispose();
            sphereSamples.Dispose();
            //randomDirections.Dispose();
            volumePlanes.Dispose();
            
            foreach (var c in geoFront)
            {
                Object.DestroyImmediate(c.gameObject);
            }
            foreach (var c in geoBack)
            {
                Object.DestroyImmediate(c.gameObject);
            }
            
            //NOTE do not use max distance, instead use aabbMagnitude so distance fields are interchangeable 
            bakeComplete?.Invoke( this, distancesOut, aabbMagnitude, passthrough );
        }
        
#if USE_BURST_AND_MATH
        [BurstCompile]
#endif
        struct CalculateRayLengths : IJobParallelFor
        {
            [ReadOnly] public NativeArray<vec3> Samples;
            [ReadOnly] public NativeArray<vec4> VolumePlanes;
            [NativeDisableContainerSafetyRestriction] //NOTE should be SAFE because it is using slices
            public NativeSlice<float> RayLengths;
            public vec3 RayOrigin;
            public float RayLength; //default ray length
            
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            bool PlaneRaycast( vec3 ro, vec3 rd, vec4 plane, out float distance )
            {
#if USE_BURST_AND_MATH
                float a = dot(rd, plane.xyz);
                float num = -dot(ro, plane.xyz) - plane.w;
#else
                float a = dot(rd, plane);
                float num = -dot(ro, plane) - plane.w;
#endif
                if ( abs(a) < EPSILON )
                {
                    distance = 0.0f;
                    return false;
                }
                distance = num / a;
                return distance > 0.0;
            }
            
            public void Execute(int index)
            {
                vec3 rd = Samples[index];
                vec3 ro = RayOrigin;
                
                float nearest = RayLength;
                
                for (int v = 0; v < 6; v++)
                {
                    if ( PlaneRaycast(ro,rd,VolumePlanes[v],out float d)) 
                        nearest = min(nearest, d);
                }

                RayLengths[index] = nearest;
            }
        }
        
#if USE_BURST_AND_MATH
        [BurstCompile]
#endif
        struct PrepareRaycastCommands : IJobParallelFor
        {
            [ReadOnly] public NativeArray<vec3> Samples;
            [NativeDisableContainerSafetyRestriction] //NOTE should be SAFE because it is using slices
            [ReadOnly] public NativeSlice<float> RayLengths;
            public NativeArray<RaycastCommand> Raycasts;
            public vec3 RayOrigin;
            public int LayerMask;
            
            public void Execute(int index)
            {
                vec3 rd = Samples[index];
                vec3 ro = RayOrigin;
                                
                Raycasts[index] = new RaycastCommand(ro, rd, RayLengths[index], LayerMask );
            }
        }

        //TODO SIMD-ize
        private struct CellResults
        {
            public float FrontMinDistance;
            public int FrontTotalHits;
            public float BackMinDistance;
            public int BackTotalHits;
        }
        
#if USE_BURST_AND_MATH
        [BurstCompile]
#endif
        struct ProcessHits : IJob
        {
            [NativeDisableContainerSafetyRestriction]
            [ReadOnly] public NativeSlice<RaycastHit> FrontHits;
            [NativeDisableContainerSafetyRestriction]
            [ReadOnly] public NativeSlice<RaycastHit> BackHits;
            [NativeDisableContainerSafetyRestriction]
            public NativeSlice<CellResults> Results; //front and back
            
            public void Execute()
            {
                int samples = FrontHits.Length;
                int frontHits = 0;
                int backHits = 0;
                float minFront = float.MaxValue;
                float minBack = float.MaxValue;
                
                for (int i = 0; i < samples; i++)
                {
                    RaycastHit frontHit = FrontHits[i];
                    RaycastHit backHit = BackHits[i];
                    
                    float frontHitDistance = abs(frontHit.distance);
                    float backHitDistance = abs(backHit.distance);
                    
                    //workaround to avoid using Collider (main thread only)
                    bool didFrontHit = frontHitDistance > 0f;
                    bool didBackHit = backHitDistance > 0f;

                    if (didFrontHit)
                    {
                        if (didBackHit && backHitDistance < frontHitDistance)
                        {
                            backHits++;
                            minBack = min(minBack, backHitDistance);
                        }
                        else
                        {
                            frontHits++;
                            minFront = min(minFront, frontHitDistance);
                        }
                    }
                    else if ( didBackHit )
                    {
                        backHits++;
                        minBack = min(minBack, backHitDistance);
                    }
                }
                
                CellResults results;
                results.FrontTotalHits = frontHits;
                results.FrontMinDistance = minFront;
                results.BackTotalHits = backHits;
                results.BackMinDistance = minBack;
                //using native slice, only write to the one index allowed!
                Results[0] = results;
            }
        }

        struct CompareDistances : IJobParallelFor
        {
            [ReadOnly] 
            public NativeArray<CellResults> Results;
            public NativeArray<float> Distances;
            
            public void Execute( int index )
            {
                //now based on results determine if cell is front or back facing
                CellResults results = Results[index];
                
                int combinedHits = results.FrontTotalHits + results.BackTotalHits;
                
                float finalDistance = 0f;
                float sign = 1f;
                float fh = max(0.000001f, results.FrontTotalHits);
                float bh = max(0.000001f, results.BackTotalHits);
                
                //NOTE only take if combined hits are greater than 0, otherwise no hit
                if (combinedHits > 0)
                {
                    if (fh / bh >= 1f ) //more front hits
                    {
                        finalDistance = results.FrontMinDistance;
                        sign = 1f;
                    }
                    else //more back
                    {
                        finalDistance = results.BackMinDistance;
                        sign = -1f;
                    }
                }

                //extra safety
                if (float.IsInfinity(finalDistance) || float.IsNaN(finalDistance))
                {
                    finalDistance = float.MaxValue;
                }
                
                //store signed distance
                Distances[index] = finalDistance * sign;
            }
        }
        
        private static void CreateColliders( ref List<Renderer> renderers, ref List<Collider> geoFront, ref List<Collider> geoBack)
        {
            foreach (var r in renderers)
            {
                Mesh mesh = null;
                if (r is MeshRenderer)
                {
                    MeshFilter f = r.GetComponent<MeshFilter>();
                    if (f == null) continue;
                    if (f.sharedMesh == null) continue;
                    mesh = f.sharedMesh;
                }
                else if (r is SkinnedMeshRenderer)
                {
                    if ((r as SkinnedMeshRenderer).sharedMesh == null) continue;
                    mesh = (r as SkinnedMeshRenderer).sharedMesh;
                }
                else
                {
                    continue;
                }

                //the renderer's transform
                Transform t = r.transform;
                vec3 tPosition = t.position;
                Quaternion tRotation = t.rotation;
                vec3 tScale = t.lossyScale;

                //front facing collision
                GameObject front = new GameObject {layer = LAYER_FRONT_GEO};
                front.transform.position = tPosition;
                front.transform.rotation = tRotation;
                front.transform.localScale = tScale;
                MeshCollider frontCollider = front.AddComponent<MeshCollider>();
                frontCollider.cookingOptions = MeshColliderCookingOptions.None;
                frontCollider.convex = false;
                frontCollider.sharedMesh = mesh;
                geoFront.Add( frontCollider );

                //back facing collision
                Mesh flippedMesh = FlipTriangles( mesh );
                GameObject back = new GameObject {layer = LAYER_BACK_GEO};
                back.transform.position = tPosition;
                back.transform.rotation = tRotation;
                back.transform.localScale = tScale;
                MeshCollider backCollider = back.AddComponent<MeshCollider>();
                backCollider.cookingOptions = MeshColliderCookingOptions.None;
                backCollider.convex = false;
                backCollider.sharedMesh = flippedMesh;
                geoBack.Add( backCollider );
            }
        }

        private static Mesh FlipTriangles(Mesh mesh)
        {
            if (mesh == null) return null;

            Mesh flipped = new Mesh
            {
                vertices = mesh.vertices,
                uv = mesh.uv,
                uv2 = mesh.uv2,
                subMeshCount = mesh.subMeshCount
            };

            for (int s = 0; s < mesh.subMeshCount; s++)
            {
                int[] triangles = mesh.GetTriangles(s);
                for (int i = 0; i < triangles.Length / 3; i++)
                {
                    int a = triangles[i * 3 + 0];
                    int c = triangles[i * 3 + 2];
                    triangles[i * 3 + 0] = c;
                    triangles[i * 3 + 2] = a;
                }

                flipped.SetTriangles(triangles, s);
            }

            flipped.RecalculateNormals();

            return flipped;
        }

#if USE_BURST_AND_MATH
        [BurstCompile]
#endif
        private static void GetUniformPointsOnSphereNormalized( ref NativeArray<vec3> samples )
        {
            int count = samples.Length;
            float fPoints = count;
            float pi = PI;
            float inc = pi * (3f - sqrt(5f));
            float off = 2f / fPoints;
		
            for (int k = 0; k < count; k++)
            {
                float y = k * off - 1f + (off / 2f);
                float r = sqrt(1f - y * y);
                float phi = k * inc;

                samples[k] = normalize(new vec3(cos(phi) * r, y, sin(phi) * r));
            }
        }

#if USE_BURST_AND_MATH
        [BurstCompile]
#endif
        private static void GetRandomDirections( vec3 halfVoxel, uint seed, ref NativeArray<vec3> samples)
        {
            int count = samples.Length;

#if USE_BURST_AND_MATH
            Random random = new Random(seed);
#endif
            for (int k = 0; k < count; k++)
            {
#if USE_BURST_AND_MATH
                samples[k] = random.NextFloat3Direction() * halfVoxel;
#else
                samples[k] = Vector3.Scale(Random.onUnitSphere,halfVoxel);
#endif
            }

        }
        
#if USE_BURST_AND_MATH
        private static readonly float PI = (float)math.PI; 
        private static readonly float EPSILON = math.FLT_MIN_NORMAL;
#else
        private const float PI = Mathf.PI;
        private static readonly float EPSILON = Mathf.Epsilon;
#endif      

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float max(float a, float b)
        {
#if USE_BURST_AND_MATH
            return math.max(a,b);  
#else
            return Mathf.Max(a, b);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float min(float a, float b)
        {
#if USE_BURST_AND_MATH
            return math.min(a,b);  
#else
            return Mathf.Min(a, b);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int min(int a, int b)
        {
#if USE_BURST_AND_MATH
            return math.min(a,b);  
#else
            return Mathf.Min(a, b);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float abs(float a)
        {
#if USE_BURST_AND_MATH
            return math.abs(a);  
#else
            return Mathf.Abs(a);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float sqrt(float a)
        {
#if USE_BURST_AND_MATH
            return math.sqrt(a);  
#else
            return Mathf.Sqrt(a);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float sin(float a)
        {
#if USE_BURST_AND_MATH
            return math.sin(a);  
#else
            return Mathf.Sin(a);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float cos(float a)
        {
#if USE_BURST_AND_MATH
            return math.cos(a);  
#else
            return Mathf.Cos(a);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float dot(vec3 a, vec3 b)
        {
#if USE_BURST_AND_MATH
            return math.dot(a,b);  
#else
            return Vector3.Dot(a,b);
#endif
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static vec3 normalize(vec3 a)
        {
#if USE_BURST_AND_MATH
            return math.normalize(a);  
#else
            return Vector3.Normalize(a);
#endif
        }
        
    }
}