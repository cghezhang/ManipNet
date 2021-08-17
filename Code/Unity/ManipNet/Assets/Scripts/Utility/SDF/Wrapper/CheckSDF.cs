using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SDFr
{	
	[RequireComponent(typeof(SDFBaker))]
	[ExecuteInEditMode]
	public class CheckSDF : MonoBehaviour
	{	
		// for voxel/sdf/gradient extraction/visualization
		public bool drawPivot = false;
		public bool drawDistance = false;
		public bool drawGradient = false;
		public bool drawPointCloud = false;
		public float slideMaxValue = 1f;
		public float slideMinValue = -1f;
		public Vector3Int dim =  new Vector3Int(16, 16, 16);

		// for nearest surface point extraction.
		public bool drawKNNVoxels = false;
		public bool drawKNNSurfacePoints = false;
		public bool drawTargetPoint = false;
		public bool drawDeepestPoint = false;
		public GameObject ball;
		public GameObject ball1;
		public GameObject capsule;
		public int K=0;
		public bool DoubleOperation = false;

		// for storing the order of NN voxel
		public Vector3Int[] NNOrder;
		public bool drawNNVoxelViaNewSearch = false;
		public bool drawKNNVoxelViaNewSearch = false;


		[SerializeField] private Matrix4x4 cubeLocalTransformation;
		[SerializeField] private CuboidMap cuboidMap = null;
		[SerializeField] private float[] realDistance;
		[SerializeField] private Vector3[] realGradient; // facing the norm direction no matter inside or outside 
		[SerializeField] private float realMaxValue;
		[SerializeField] private float realMinValue;

		public CuboidMap GetCuboidMap(){
			return cuboidMap;
		}
		public float[] GetRealDistances(){
			float[] dis = new float[realDistance.Length];
			for(int i=0; i<realDistance.Length; i++){
				dis[i] = realDistance[i];
			}
			return dis;
		}
		public Vector3[] GetRealGradients(){
			Vector3[] gra = new Vector3[realGradient.Length];
			for(int i=0; i<realGradient.Length; i++){
				gra[i] = GetWorldGradient(i);
			}
			return gra;
		}
		public Vector3 GetWorldGradient(int index){
			/**
			the gradient need to be mirrored as well if the current sdf is mirrored 
			*/
			return realGradient[index].GetMirror(GetCurrentMirrorAxis()).GetRelativeDirectionFrom(cuboidMap.Pivot);
		}


		public SDFBaker GetSDFBaker(){
			SDFBaker baker = (SDFBaker)GetComponent<SDFBaker>();
			return baker;
		}

		public Bounds GetBounds(){
			SDFBaker baker = (SDFBaker)GetSDFBaker();
			if(baker!=null){
				return baker.GetBounds();
			}
			return new Bounds();
		}

		public Axis GetCurrentMirrorAxis(){
			/**
			this is for getting current object world mirror situation, and used for mirroring local transformations for cuboid
			because if cube mirror the cube local position and gradient need to be mirrored as well
			*/
			Vector3 lossyScal = transform.lossyScale;
			return lossyScal.GetMirrorAxis();
		}
		public Matrix4x4 GetCubeLocalTransformation(Axis mirrorAxis){
			if(cubeLocalTransformation!=null){
				return cubeLocalTransformation.GetMirror(mirrorAxis);
			}
			Debug.Log("no cube local transformation");
			return Matrix4x4.identity;
		}
		public Matrix4x4 GetCubeWorldTransformation(){
			/**
			first need to judge the mirror situation, later maybe need to consider the scale for resize the object
			then the object transform need to use unit scale because when assign the position and rotation, we already gives the mirrored position and rotation,
			if using lossyscale, it will mirror the mirrored transformation
			*/
			
			Matrix4x4 local = GetCubeLocalTransformation(GetCurrentMirrorAxis());
			if(local!=null){
				return local.GetRelativeTransformationFrom(transform.GetWorldMatrix(true)); 
			}
			Debug.Log("no cube local transformation");
			return Matrix4x4.identity;
		}

		public MeshCollider GetMeshCollider(){
			return GetComponentInChildren<MeshCollider>();
		}
		public Vector3 GetMeshCenter(){
			return GetMeshCollider().bounds.center;
		}
		public Vector3 GetMeshSize(){
			return GetMeshCollider().bounds.size;
		}

		
		#if UNITY_EDITOR
		private void ExtractVoxel(){
			SDFBaker baker = GetSDFBaker();
			if(baker != null){
				baker.SetDimension(dim);
				baker.Bake(); // rebake
				
				// change the order
				float[] distance = baker.GetDistances();  
				float[,,] distance3D = new float[dim.x, dim.y, dim.z];
				realMaxValue = 0;
				realMinValue = 0;
				for ( int x = 0; x < dim.x; x++ )
				{
					for ( int y = 0; y < dim.y; y++ )
					{
						for ( int z = 0; z < dim.z; z++ )
						{
							float dist = distance[x * dim.y * dim.z + y * dim.z + z];
							distance3D[z,y,x] = dist;
							if(dist<realMinValue){
								realMinValue = dist;
							}
							if(dist>realMaxValue){
								realMaxValue = dist;
							}
						}
					}
				}
				Debug.Log(realMinValue);
				Debug.Log(realMaxValue);

				// get the real distance and real gradient
				realDistance = new float[dim.x*dim.y*dim.z];
				realGradient = new Vector3[dim.x*dim.y*dim.z];
				float[] normdDistance = new float[dim.x*dim.y*dim.z];
				for ( int x = 0; x < dim.x; x++ )
				{
					for ( int y = 0; y < dim.y; y++ )
					{
						for ( int z = 0; z < dim.z; z++ )
						{
							realDistance[x * dim.y * dim.z + y * dim.z + z] =  distance3D[x,y,z];
							normdDistance[x * dim.y * dim.z + y * dim.z + z] = distance3D[x,y,z]/Mathf.Abs(realMinValue);
						}
					}
				}
				
				// initial the cuboidmap
				cuboidMap = new CuboidMap(dim);
				Matrix4x4 center = transform.GetWorldMatrix();
				Bounds bounds = baker.GetBounds();
				center[0,3] += bounds.center.x;
				center[1,3] += bounds.center.y;
				center[2,3] += bounds.center.z;
				cuboidMap.Size = bounds.size;
				cuboidMap.Retransform(center, GetCurrentMirrorAxis());
				cuboidMap.Occupancies = normdDistance;

				// here extract the gradient, but the gradient need to relative to the pivot, this is important 
				realGradient = ExtractGradient(realDistance, cuboidMap.Pivot);

				// set local trasformation for movement, relative to the object transformation
				cubeLocalTransformation = center.GetRelativeTransformationTo(transform.GetWorldMatrix());
			}
			else{
				Debug.Log("bakers are not found!");
			}
		}

		private Vector3[] ExtractGradient(float[] realDistance, Matrix4x4 pivot){
			Vector3[] grads = new Vector3[dim.x * dim.y * dim.z];
			for(int x = 0; x < dim.x; x++){
				for(int y = 0; y < dim.y; y++){
					for(int z = 0; z < dim.z; z++){
						float d = realDistance[x * dim.y * dim.z + y * dim.z + z];
						float sign = d >= 0 ? 1.0f : -1.0f;
						float maxval = float.MaxValue * sign;

						//read neighbour distances, ignoring border pixels
						float x0 = x > 0 ? realDistance[(x-1) * dim.y * dim.z + y * dim.z + z] : maxval;
						float x1 = x < (dim.x - 1) ? realDistance[(x+1) * dim.y * dim.z + y * dim.z + z] : maxval;
						float y0 = y > 0 ? realDistance[x * dim.y * dim.z + (y-1) * dim.z + z] : maxval;
						float y1 = y < (dim.y - 1) ? realDistance[x * dim.y * dim.z + (y+1) * dim.z + z] : maxval;
						float z0 = z > 0 ? realDistance[x * dim.y * dim.z + y * dim.z + (z-1)] : maxval;
						float z1 = z < (dim.z - 1) ? realDistance[x * dim.y * dim.z + y * dim.z + (z+1)] : maxval;

						//use the smallest neighbour in each direction to calculate the partial deriviates
						float xgrad = sign*x0 < sign*x1 ? -(x0-d) : (x1-d);
						float ygrad = sign*y0 < sign*y1 ? -(y0-d) : (y1-d);
						float zgrad = sign*z0 < sign*z1 ? -(z0-d) : (z1-d);

						// here again is important to get relative directions
						grads[x * dim.y * dim.z + y * dim.z + z] = new Vector3(xgrad, ygrad, zgrad).GetRelativeDirectionTo(pivot);
					}
				}
			}
			return grads;
		}
		#endif

		private void OnRenderObject() {
			if(drawPivot){
				if(cuboidMap != null){
					UltiDraw.Begin();
					UltiDraw.DrawLine(cuboidMap.Pivot.GetPosition(), cuboidMap.Pivot.GetPosition() + 1f*cuboidMap.Pivot.GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.75f));
					UltiDraw.DrawLine(cuboidMap.Pivot.GetPosition(), cuboidMap.Pivot.GetPosition() + 1f*cuboidMap.Pivot.GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.75f));	
					UltiDraw.DrawLine(cuboidMap.Pivot.GetPosition(), cuboidMap.Pivot.GetPosition() + 1f*cuboidMap.Pivot.GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.75f));
				
					UltiDraw.End();
				}
				else{
					Debug.Log("cuboidMap is null");
				}
			}
			if(drawDistance){
				if(cuboidMap != null){
					UltiDraw.Begin();
					for(int i=0; i<realDistance.Length; i++){
						if(cuboidMap.Occupancies[i]<slideMaxValue && cuboidMap.Occupancies[i]>slideMinValue){
							UltiDraw.DrawCuboid(cuboidMap.References[i], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Black.Transparent(0.5f));
						}
					}
					UltiDraw.End();
				}
				else{
					Debug.Log("cuboidMap is null");
				}
			}
			if(drawGradient){
				if(cuboidMap != null){
					UltiDraw.Begin();
					for(int i=0; i<realGradient.Length; i++){
						if(cuboidMap.Occupancies[i]<slideMaxValue && cuboidMap.Occupancies[i]>slideMinValue ){
							Vector3 direction = GetWorldGradient(i).normalized; 
							// 0.2 here is scale of the arrow
							UltiDraw.DrawArrow(cuboidMap.References[i], cuboidMap.References[i]+direction * 0.05f, 0.5f, 0.005f, 0.01f, UltiDraw.Red.Transparent(0.4f));
						}
					}
					UltiDraw.End();
				}
				else{
					Debug.Log("cuboidMap is null");
				}
			}
			// point cloud is generated from the distance and gradient (selected from min and max value)
			if(drawPointCloud){
				if(cuboidMap != null){
					UltiDraw.Begin();
					for(int i=0; i<realGradient.Length; i++){
						if(cuboidMap.Occupancies[i]<slideMaxValue && cuboidMap.Occupancies[i]>slideMinValue ){
							Vector3 direction = GetWorldGradient(i).normalized;
							UltiDraw.DrawSphere(cuboidMap.References[i]-direction*realDistance[i], Quaternion.identity, 0.05f, UltiDraw.Black.Transparent(0.5f));
						}
					}
					UltiDraw.End();
				}
				else{
					Debug.Log("cuboidMap is null");
				}
			}
			if(ball!=null && K>0 && (drawTargetPoint || drawKNNVoxels || drawKNNSurfacePoints)){
				if(ball1!=null){
					// get nearest point on the line-segment
					Vector3 startP = ball.transform.position;
					Vector3 endP = ball1.transform.position;
					int[] indices;
					Vector3 targetPoint = FindPointViaKNNSDF(startP, endP, out indices, K, 0.5f, -0.5f);
					// here could use two loops, the first time project the point near the surface
					if(DoubleOperation){
						targetPoint = FindPointViaKNNSDF(targetPoint, out indices, K, 0f, -0.5f);
					}
					if(indices!=null){
						UltiDraw.Begin();
						for(int i=0; i<indices.Length; i++){
							int index = indices[i];
							Vector3 surfacePoint = cuboidMap.References[index]-GetWorldGradient(index).normalized*realDistance[index];
							if(drawKNNVoxels){
								UltiDraw.DrawCuboid(cuboidMap.References[index], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Red.Transparent(0.8f));
							}
							if(drawKNNSurfacePoints){
								UltiDraw.DrawSphere(surfacePoint, Quaternion.identity, cuboidMap.GetStep().magnitude/2f, UltiDraw.Black);
							}
						}
						if(drawTargetPoint){
							UltiDraw.DrawSphere(targetPoint, Quaternion.identity, cuboidMap.GetStep().magnitude/2f, UltiDraw.Yellow);
							Vector3 segP = Utility.ClosestPointOnLineSegment(startP, endP, targetPoint);
							UltiDraw.DrawSphere(segP, Quaternion.identity, cuboidMap.GetStep().magnitude/2f, UltiDraw.Red);
						}
						UltiDraw.End();
					}
				}
				else{
					Vector3 ballP = ball.transform.position;
					int[] indices;
					float signedD;
					Vector3 targetPoint = FindPointViaKNNSDF_new(ballP, out indices, out signedD, K, 0.6f, -0.6f);
					// Vector3 targetPoint = FindPointViaKNNSDF(ballP, out indices, K, 0f, -0.5f);
					// here could use two loops, the first time project the point near the surface
					// if(DoubleOperation){
					// 	targetPoint = FindPointViaKNNSDF(targetPoint, out indices, K, 0f, -0.5f);
					// }
					if(indices!=null){
						UltiDraw.Begin();
						for(int i=0; i<indices.Length; i++){
							int index = indices[i];
							Vector3 surfacePoint = cuboidMap.References[index]-GetWorldGradient(index).normalized*realDistance[index];
							if(drawKNNVoxels){
								UltiDraw.DrawCuboid(cuboidMap.References[index], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Black.Transparent(0.8f));
							}
							if(drawKNNSurfacePoints){
								UltiDraw.DrawSphere(surfacePoint, Quaternion.identity, cuboidMap.GetStep().magnitude/2f, UltiDraw.Black);
							}
							UltiDraw.DrawCuboid(cuboidMap.References[0], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Red.Transparent(0.8f));
							
							UltiDraw.DrawCuboid(cuboidMap.References[1], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Green.Transparent(0.8f));
							
							UltiDraw.DrawCuboid(cuboidMap.References[10], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Blue.Transparent(0.8f));

							
							UltiDraw.DrawCuboid(cuboidMap.References[100], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Yellow.Transparent(0.8f));
							
						}
						if(drawTargetPoint){
							UltiDraw.DrawSphere(targetPoint, Quaternion.identity, cuboidMap.GetStep().magnitude/2f, UltiDraw.Yellow);
						}
						UltiDraw.End();
					}
				}
			}

			if(ball!=null && drawNNVoxelViaNewSearch){
				Vector3 ballP = ball.transform.position;
				Vector3 boundSize = GetBounds().size;
				Matrix4x4 cornerTrans = GetCornerTrans(cuboidMap.Pivot, boundSize);
				int index = FindNNVoxelViaNewSearch(ballP,cornerTrans, boundSize, dim);
				UltiDraw.Begin();
				UltiDraw.DrawLine(cornerTrans.GetPosition(), cornerTrans.GetPosition() + 1f*cornerTrans.GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.75f));
				UltiDraw.DrawLine(cornerTrans.GetPosition(), cornerTrans.GetPosition() + 1f*cornerTrans.GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.75f));	
				UltiDraw.DrawLine(cornerTrans.GetPosition(), cornerTrans.GetPosition() + 1f*cornerTrans.GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.75f));
				UltiDraw.End();

				UltiDraw.Begin();
				UltiDraw.DrawCuboid(cuboidMap.References[index], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Black.Transparent(0.8f));
				UltiDraw.End();
			}

			if(ball!=null && drawKNNVoxelViaNewSearch){
				Vector3 ballP = ball.transform.position;
				Vector3 boundSize = GetBounds().size;
				int[] indices = FindKNNVoxelsViaNewSearch(ballP, GetCornerTrans(cuboidMap.Pivot, boundSize), boundSize, dim, K);
				Matrix4x4 cornerTrans = GetCornerTrans(cuboidMap.Pivot, GetBounds().size);
				UltiDraw.Begin();
				UltiDraw.DrawLine(cornerTrans.GetPosition(), cornerTrans.GetPosition() + 1f*cornerTrans.GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.75f));
				UltiDraw.DrawLine(cornerTrans.GetPosition(), cornerTrans.GetPosition() + 1f*cornerTrans.GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.75f));	
				UltiDraw.DrawLine(cornerTrans.GetPosition(), cornerTrans.GetPosition() + 1f*cornerTrans.GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.75f));
				UltiDraw.End();

				UltiDraw.Begin();
				for(int i=0; i<indices.Length; i++){
					UltiDraw.DrawCuboid(cuboidMap.References[indices[i]], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Black.Transparent(0.8f));
				}
				UltiDraw.End();
			}


			if(capsule!=null && K>0 && drawDeepestPoint){
				CapsuleCollider capsuleCollider = capsule.GetComponent<CapsuleCollider>();
				if(capsuleCollider!= null){
					int[] indices = new int[0];
					Vector3 deepestPoint = FindDeepestPointViaSDF(capsuleCollider, out indices, K, 1f);
					UltiDraw.Begin();
					UltiDraw.DrawSphere(deepestPoint, Quaternion.identity, cuboidMap.GetStep().magnitude/5f, UltiDraw.Yellow);
					for(int i=0; i<indices.Length; i++){
						int index = indices[i];
						if(drawKNNVoxels){
							UltiDraw.DrawCuboid(cuboidMap.References[index], cuboidMap.Pivot.GetRotation(), cuboidMap.GetStep(), UltiDraw.Red.Transparent(0.8f));
						}
					}
					UltiDraw.End();
				}
			}
		}

		// ============== for new search method ==================
		public Matrix4x4 GetCornerTrans(Matrix4x4 pivotTrans, Vector3 boundSize){
			/*
			input the center of sdf and the size of its bound
			output the corner transformation of the bound around the first voxel
			*/
			Matrix4x4 cornerTrans = pivotTrans;
			Matrix4x4 localTrans = Matrix4x4.TRS(-boundSize/2f, Quaternion.identity, Vector3.one).GetMirror(GetCurrentMirrorAxis());
			cornerTrans = cornerTrans * localTrans;
			return cornerTrans;
		}
		public int FindNNVoxelViaNewSearch(Vector3 point, Matrix4x4 cornerTrans, Vector3 boundSize, Vector3Int resolution){
			// transform the point into cornerTrans
			point = point.GetRelativePositionTo(cornerTrans).GetMirror(GetCurrentMirrorAxis());
			Vector3 scaledPosition = new Vector3(Mathf.Clamp(point.x/(boundSize.x/(float)resolution.x), 0f, (float)resolution.x-0.1f), 
												 Mathf.Clamp(point.y/(boundSize.y/(float)resolution.y), 0f, (float)resolution.y-0.1f), 
												 Mathf.Clamp(point.z/(boundSize.z/(float)resolution.z), 0f, (float)resolution.z-0.1f));
			Vector3Int voxelIndex = new Vector3Int((int)scaledPosition.x, (int)scaledPosition.y, (int)scaledPosition.z);
			return voxelIndex.z + voxelIndex.y * resolution.z + voxelIndex.x * resolution.y * resolution.z;
		}

		public Vector3Int FindNNVoxel3DViaNewSearch(Vector3 point, Matrix4x4 cornerTrans, Vector3 boundSize, Vector3Int resolution){
			// transform the point into cornerTrans
			Vector3Int index = Vector3Int.zero;
			point = point.GetRelativePositionTo(cornerTrans).GetMirror(GetCurrentMirrorAxis());
			Vector3 scaledPosition = new Vector3(Mathf.Clamp(point.x/(boundSize.x/(float)resolution.x), 0f, (float)resolution.x-0.1f), 
												 Mathf.Clamp(point.y/(boundSize.y/(float)resolution.y), 0f, (float)resolution.y-0.1f), 
												 Mathf.Clamp(point.z/(boundSize.z/(float)resolution.z), 0f, (float)resolution.z-0.1f));
			index = new Vector3Int((int)scaledPosition.x, (int)scaledPosition.y, (int)scaledPosition.z);
			return index;
		}

		public int[] FindKNNVoxelsViaNewSearch(Vector3 point, Matrix4x4 cornerTrans, Vector3 boundSize, Vector3Int resolution, int K){
			// check the NN Order
			if(NNOrder== null || !(NNOrder.Length>0)){
				NNOrder = InitializeNNOrder(3).ToArray(); // here we use 3, so 7*7*7 is enough
			}
			// check the K number
			if(K>50){
				Debug.LogError("K seems too big");
				return null;
			}

			int[] indices = new int[K];
			Vector3Int center = FindNNVoxel3DViaNewSearch(point, cornerTrans, boundSize, resolution);
			
			{// here get the K near voxel indices by following NNOrder
				int KIndex=0; 
				int ArrIndex=0;  
				while(KIndex < K){
					Vector3Int currentIndex = center+NNOrder[ArrIndex];
					if(currentIndex.x>=0 && currentIndex.x<resolution.x && currentIndex.y>=0 && currentIndex.y<resolution.y && currentIndex.z>=0 && currentIndex.z<resolution.z){
						// then valid
						indices[KIndex] = currentIndex.z + currentIndex.y * resolution.z + currentIndex.x * resolution.y * resolution.z;
						KIndex++;
					}
					ArrIndex++;
				}
			}
			
			return indices;
		}

		public List<Vector3Int> InitializeNNOrder(int N){
			int distance = 2*N+1;
			List<Vector3Int> results = new List<Vector3Int>();
			for(int i=-N; i<=N; i++){
				for(int j=-N; j<=N; j++){
					for(int k=-N; k<=N; k++){
						results.Add(new Vector3Int(i,j,k));
					}
				}
			}
			results = results.OrderBy(vec => vec.magnitude).ToList();
			return results;
		}
		// ============== for new search method ==================

		public Vector3 FindDeepestPointViaSDF(Vector3 startP, Vector3 endP){
			/// <summary>
			/// todo implement for the line segments
			/// </summary>
			return Vector3.zero;
		}
		public Vector3 FindDeepestPointViaSDF(CapsuleCollider jointCollider, out int[] kIndices, int K=20, float distanceThreshold = 1f){
			/// <summary>
			/// get the K-NN voxels that also has smallest realDistance value(more inner)
			/// </summary>
			kIndices = new int[K];
			if(cuboidMap != null){
				float[] distances = new float[cuboidMap.References.Length];
				for(int i=0; i<distances.Length; i++){
					// here choose a small value 
					if(cuboidMap.Occupancies[i]<distanceThreshold){
						Vector3 capsulePoint =  jointCollider.ClosestPoint(cuboidMap.References[i]);
						distances[i] = (capsulePoint-cuboidMap.References[i]).sqrMagnitude;
					}
					else{
						distances[i] = float.MaxValue;
					}
				}
				// get the indices of KNN voxels that is really inside the objects
				kIndices = FindKNNSmallIndices(distances, realDistance, K, 0.05f);
				// kIndices = FindKNNIndices(distances, K);
				// then blend the voxels by the distance
				float[] kDistances = new float[K];
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					kDistances[i] = distances[kIndices[i]];
				}
				// get the blending weights from the surfaceDistance
				float[] voxelBlendWeights = new float[K];
				float maxDistance = kDistances.Max();
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					voxelBlendWeights[i] = maxDistance + 1/50f - kDistances[i];
				}
				Vector3 voxelP = Vector3.zero;
				for(int i=0; i<K; i++){
					// here get the deepest voxel
					voxelP += voxelBlendWeights[i]/voxelBlendWeights.Sum() * cuboidMap.References[kIndices[i]];
				}
				return jointCollider.ClosestPoint(voxelP);
			}
			return Vector3.zero;
		}

		
		public Vector3 FindPointViaKNNSDF_new(Vector3 currentPosition, out int[] kIndices, out float signedDistance,
										      int K = 3, float upValue = 1f, float lowValue = -1f){
			/// <summary>
			/// here we firstly find the KNN voxels, then we blend their position and gradient, then we return the surface point and KNN indices and distance
			/// </summary>
			Vector3 surfacePoint = currentPosition;
			kIndices = new int[K];
			signedDistance = 0f;
			if(cuboidMap != null){
				// calculate the distances and filter out voxels out of range by normalized distance
				float[] distances = new float[cuboidMap.References.Length];
				for(int i=0; i<distances.Length; i++){
					if(cuboidMap.Occupancies[i]>lowValue && cuboidMap.Occupancies[i]<upValue){
						// here use sqrMag for speed up, but value is not the distance
						distances[i] = (currentPosition-cuboidMap.References[i]).sqrMagnitude;
					}
					else{
						distances[i] = float.MaxValue;
					}
				}
				// get the indices of KNN voxels
				kIndices = FindKNNIndices(distances, K);

				// get the K distance and gradient
				float[] KDistances = new float[K];
				float[] KRealDistances = new float[K];
				Vector3[] KRealGradients = new Vector3[K];
				for(int i=0; i<K; i++){
					int voxelIndex = kIndices[i];
					KDistances[i] = (cuboidMap.References[voxelIndex]-currentPosition).magnitude;
					KRealDistances[i] = realDistance[voxelIndex];
					KRealGradients[i] = GetWorldGradient(voxelIndex);
				}
				// rebuild the blending weights
				float maxDistance = KDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				float[] rebuiltWeights = new float[K];
				for(int i=0; i<K; i++){
					rebuiltWeights[i] = maxDistance + voxelDignalDistance/50f - KDistances[i];
				}
				Vector3 blendedPosition = Vector3.zero;
				for(int i=0; i<K; i++){
					blendedPosition += rebuiltWeights[i]/rebuiltWeights.Sum()*cuboidMap.References[kIndices[i]] ;
				}
				float finalDistance = 0f;
				Vector3 finalGradient = Vector3.zero;
				for(int i=0; i<K; i++){
					finalDistance += rebuiltWeights[i]/rebuiltWeights.Sum() * KRealDistances[i];
					finalGradient += rebuiltWeights[i]/rebuiltWeights.Sum() * KRealGradients[i];
				}
				surfacePoint = blendedPosition-finalGradient.normalized*finalDistance;
				signedDistance = finalDistance<0 ? -(currentPosition-surfacePoint).magnitude:(currentPosition-surfacePoint).magnitude;
			}
			else{
				Debug.Log("cuboidMap is null");
			}
			return surfacePoint;
		}


		public Vector3 FindPointViaKNNSDF(Vector3 startSeg, Vector3 endSeg, out int[] kIndices, 
												 int K = 3, float upValue = 1f, float lowValue = -1f,
												 float psDistance = 0f){
			Vector3 targetPosition = (startSeg+endSeg)/2f;
			kIndices = new int[K];
			if(cuboidMap != null){
				// calculate the distances and filter out voxels out of range by normalized distance
				float[] distances = new float[cuboidMap.References.Length];
				for(int i=0; i<distances.Length; i++){
					if(cuboidMap.Occupancies[i]>lowValue && cuboidMap.Occupancies[i]<upValue){
						Vector3 closestPoint =  Utility.ClosestPointOnLineSegment(startSeg, endSeg, cuboidMap.References[i]);
						// here use sqrMag for speed up, but value is not the distance
						distances[i] = (closestPoint-cuboidMap.References[i]).sqrMagnitude;
					}
					else{
						distances[i] = float.MaxValue;
					}
				}
				// get the indices of KNN voxels
				kIndices = FindKNNIndices(distances, K);
				// get surface points and distance to the currentPoint
				Vector3[] surfacePoints = new Vector3[K];
				float[] surfaceDistances = new float[K];
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					if(index<realDistance.Length){
						surfacePoints[i] = cuboidMap.References[index]-GetWorldGradient(index).normalized*(realDistance[index]-psDistance);
						Vector3 closestPoint = Utility.ClosestPointOnLineSegment(startSeg, endSeg, surfacePoints[i]);
						surfaceDistances[i] = (surfacePoints[i]-closestPoint).magnitude;
					}
					else{
						Debug.LogError("Can not find KNN points");
						return targetPosition;
					}
				}

				// get the blending weights from the surfaceDistance
				float[] surfaceBlendWeights = new float[K];
				float maxDistance = surfaceDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					surfaceBlendWeights[i] = maxDistance + voxelDignalDistance/50f - surfaceDistances[i];
				}
				// interpolate the surface points to get the final one
				targetPosition = Vector3.zero;
				//surfaceDistances.Print();
				for(int i=0; i<K; i++){
					targetPosition += surfaceBlendWeights[i]/surfaceBlendWeights.Sum() * surfacePoints[i];
				}
			}
			else{
				Debug.Log("cuboidMap is null");
			}
			return targetPosition;
		}

		/// <summary>
		///	find the surface point from K nearest voxels
		/// </summary>
		/// <param name="currentPosition"></param>
		/// <param name="K"> number of the voxels that interpolated</param>
		/// <param name="upValue"> constraint the voxels that have smaller scaled values than upValue </param>
		/// <param name="lowValue"> constraint the voxels that have bigger scaled values than lowValue </param>
		/// <returns></returns>
		public Vector3 FindPointViaKNNSDF(Vector3 currentPosition, out int[] kIndices,   
										  int K = 3, float upValue = 1f, float lowValue = -1f, 
										  float psDistance = 0f){
			Vector3 targetPosition = currentPosition;
			kIndices = new int[K];
			if(cuboidMap != null){
				// calculate the distances and filter out voxels out of range by normalized distance
				float[] distances = new float[cuboidMap.References.Length];
				for(int i=0; i<distances.Length; i++){
					if(cuboidMap.Occupancies[i]>lowValue && cuboidMap.Occupancies[i]<upValue){
						// here use sqrMag for speed up, but value is not the distance
						distances[i] = (currentPosition-cuboidMap.References[i]).sqrMagnitude;
					}
					else{
						distances[i] = float.MaxValue;
					}
				}
				// get the indices of KNN voxels
				kIndices = FindKNNIndices(distances, K);
				// get surface points and distance to the currentPoint
				Vector3[] surfacePoints = new Vector3[K];
				float[] surfaceDistances = new float[K];
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					if(index<realDistance.Length){
						surfacePoints[i] = cuboidMap.References[index]-GetWorldGradient(index).normalized*(realDistance[index]-psDistance);
						surfaceDistances[i] = (currentPosition-surfacePoints[i]).magnitude;
					}
					else{
						Debug.LogError("Can not find KNN points");
						return targetPosition;
					}
				}

				// get the blending weights from the surfaceDistance
				float[] surfaceBlendWeights = new float[K];
				float maxDistance = surfaceDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					surfaceBlendWeights[i] = maxDistance + voxelDignalDistance/50f - surfaceDistances[i];
				}
				// interpolate the surface points to get the final one
				targetPosition = Vector3.zero;
				//surfaceDistances.Print();
				for(int i=0; i<K; i++){
					targetPosition += surfaceBlendWeights[i]/surfaceBlendWeights.Sum() * surfacePoints[i];
				}
			}
			else{
				Debug.Log("cuboidMap is null");
			}
			return targetPosition;
		}
		
		
		public Vector3 GetNormDirectionViaKNN(Vector3 point, int K = 10, float upValue = 1f, float lowValue = -1f){
			Vector3 dir = Vector3.zero;
			if(cuboidMap != null){
				int[] kIndices = new int[K];
				// calculate the distances and filter out voxels out of range by normalized distance
				float[] distances = new float[cuboidMap.References.Length];
				for(int i=0; i<distances.Length; i++){
					if(cuboidMap.Occupancies[i]>lowValue && cuboidMap.Occupancies[i]<upValue){
						// here use sqrMag for speed up, but value is not the distance
						distances[i] = (point-cuboidMap.References[i]).sqrMagnitude;
					}
					else{
						distances[i] = float.MaxValue;
					}
				}
				// get the indices of KNN voxels
				kIndices = FindKNNIndices(distances, K);
				
				// get the K distance and gradient
				float[] KDistances = new float[K];
				Vector3[] KRealGradients = new Vector3[K];
				for(int i=0; i<K; i++){
					int voxelIndex = kIndices[i];
					KDistances[i] = (cuboidMap.References[voxelIndex]-point).magnitude;
					KRealGradients[i] = GetWorldGradient(voxelIndex);
				}
				// rebuild the blending weights
				float maxDistance = KDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				float[] rebuiltWeights = new float[K];
				for(int i=0; i<K; i++){
					rebuiltWeights[i] = maxDistance + voxelDignalDistance/50f - KDistances[i];
				}
				for(int i=0; i<K; i++){
					dir += rebuiltWeights[i]/rebuiltWeights.Sum() * KRealGradients[i];
				}
			}
			return dir.normalized;
		}

		/// <summary>
		/// get signed distanced value for point
		/// </summary>
		/// <param name="point"></param>
		/// <param name="K"></param>
		/// <param name="upValue"></param>
		/// <param name="lowValue"></param>
		/// <returns></returns>
		public float GetSignedDistanceValueViaKNN(Vector3 point, int K = 10, float upValue = 1f, float lowValue = -1f){
			float dis = 0;
			if(cuboidMap != null){
				int[] kIndices = new int[K];
				// calculate the distances and filter out voxels out of range by normalized distance
				float[] distances = new float[cuboidMap.References.Length];
				for(int i=0; i<distances.Length; i++){
					if(cuboidMap.Occupancies[i]>lowValue && cuboidMap.Occupancies[i]<upValue){
						// here use sqrMag for speed up, but value is not the distance
						distances[i] = (point-cuboidMap.References[i]).sqrMagnitude;
					}
					else{
						distances[i] = float.MaxValue;
					}
				}
				// get the indices of KNN voxels
				kIndices = FindKNNIndices(distances, K);
				// then blend the the occupancy which indeed are signed value
				float[] KNNdistances = new float[K];
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					KNNdistances[i] = distances[index];
				}
				float[] blendWeights = new float[K];
				float maxDistance = KNNdistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					blendWeights[i] = maxDistance + voxelDignalDistance/50f - KNNdistances[i];
				}
				//surfaceDistances.Print();
				Vector3 blendedPoint = Vector3.zero;
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					dis += blendWeights[i]/blendWeights.Sum() * realDistance[index];
					blendedPoint += blendWeights[i]/blendWeights.Sum() * cuboidMap.References[index];
				}

				/// todo here is for correcting the distance if the joint is far away from the voxel
				dis = dis+(point-blendedPoint).magnitude;
			}
			return dis;
		}


		
		// ****************************************************************************************************************
		// *********************************************KNN search with KD-TREE********************************************
		// ****************************************************************************************************************
		/// <summary>
		/// get signed distanced value for point in world space
		/// </summary>
		/// <param name="point"> this point is in the world space, need to transform into cuboid pivot space</param>
		/// <param name="K"></param>
		/// <param name="upValue"></param>
		/// <param name="lowValue"></param>
		/// <returns></returns>
		public float GetSignedDistanceValueViaKDTree(Vector3 point, int K = 10){
			float dis = 0;
			KDTree.NearestNeighbour<int> neighbours = null;
			if(cuboidMap != null){
				// if non-mirror then use non-mirrored KD-tree other wise use mirrored KD-ree
				if(GetCurrentMirrorAxis() == Axis.None){
					if(cuboidMap.KDTree == null){
						cuboidMap.RebuildKDTree(GetCurrentMirrorAxis());
					}
					// transform into local space then use KD tree to get the index
					Vector3 pointLocal = point.GetRelativePositionTo(cuboidMap.Pivot);
					neighbours = cuboidMap.KDTree.NearestNeighbors(new double[3]{pointLocal.x, pointLocal.y, pointLocal.z}, K);
				}
				else{
					if(cuboidMap.KDTree_mirror == null){
						cuboidMap.RebuildKDTree(GetCurrentMirrorAxis());
					}
					// transform into local space then use KD tree to get the index
					Vector3 pointLocal = point.GetRelativePositionTo(cuboidMap.Pivot);
					neighbours = cuboidMap.KDTree_mirror.NearestNeighbors(new double[3]{pointLocal.x, pointLocal.y, pointLocal.z}, K);
				}

				int[] kIndices = new int[K];
				float[] kDistances = new float[K];
				int k = 0;
				foreach(int neighbour in neighbours) {
					kIndices[k] = neighbour;
					kDistances[k] = (point-cuboidMap.References[neighbour]).sqrMagnitude;
					k++;
                }
				float[] blendWeights = new float[K];
				float maxDistance = kDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					blendWeights[i] = maxDistance + voxelDignalDistance/50f - kDistances[i];
				}
				//surfaceDistances.Print();
				Vector3 blendedPoint = Vector3.zero;
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					dis += blendWeights[i]/blendWeights.Sum() * realDistance[index];
					blendedPoint += blendWeights[i]/blendWeights.Sum() * cuboidMap.References[index];
				}

				/// todo here is for correcting the distance if the joint is far away from the voxel
				dis = dis+(point-blendedPoint).magnitude;
			}
			return dis;
		}
		public float GetSignedDistanceValueViaKDTree(out Vector3 direction, Vector3 point, int K = 10){
			float dis = 0;
			direction = Vector3.zero;
			KDTree.NearestNeighbour<int> neighbours = null;
			if(cuboidMap != null){
				// if non-mirror then use non-mirrored KD-tree other wise use mirrored KD-ree
				if(GetCurrentMirrorAxis() == Axis.None){
					if(cuboidMap.KDTree == null){
						cuboidMap.RebuildKDTree(GetCurrentMirrorAxis());
					}
					// transform into local space then use KD tree to get the index
					Vector3 pointLocal = point.GetRelativePositionTo(cuboidMap.Pivot);
					neighbours = cuboidMap.KDTree.NearestNeighbors(new double[3]{pointLocal.x, pointLocal.y, pointLocal.z}, K);
				}
				else{
					if(cuboidMap.KDTree_mirror == null){
						cuboidMap.RebuildKDTree(GetCurrentMirrorAxis());
					}
					// transform into local space then use KD tree to get the index
					Vector3 pointLocal = point.GetRelativePositionTo(cuboidMap.Pivot);
					neighbours = cuboidMap.KDTree_mirror.NearestNeighbors(new double[3]{pointLocal.x, pointLocal.y, pointLocal.z}, K);
				}
				
				int[] kIndices = new int[K];
				float[] kDistances = new float[K];
				int k = 0;
				foreach(int neighbour in neighbours) {
					kIndices[k] = neighbour;
					kDistances[k] = (point-cuboidMap.References[neighbour]).sqrMagnitude;
					k++;
                }
				float[] blendWeights = new float[K];
				float maxDistance = kDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					blendWeights[i] = maxDistance + voxelDignalDistance/50f - kDistances[i];
				}
				//surfaceDistances.Print();
				Vector3 blendedPoint = Vector3.zero;
				Vector3 blendedGradient = Vector3.zero;
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					dis += blendWeights[i]/blendWeights.Sum() * realDistance[index];
					blendedPoint += blendWeights[i]/blendWeights.Sum() * cuboidMap.References[index];
					blendedGradient += blendWeights[i]/blendWeights.Sum() * GetWorldGradient(index); // here gradient need to mirrored as well
				}
				{
					// get surface point 
					Vector3 surfacePoint = blendedPoint - blendedGradient.normalized * dis;
					// then get the direction from input point to the surfacepoint
					direction = surfacePoint - point;
					// then get the distance from input point to the surfacepoint, have to use the following way to give a signed value
					dis = dis+(point-blendedPoint).magnitude;
				}
			}
			return dis;
		}
		// given an point, return sdf and norm direction of that point
		public float GetSignedValueAndNormDirectionViaKDTree(out Vector3 direction, Vector3 point, int K = 10){
			float dis = 0;
			direction = Vector3.zero;
			KDTree.NearestNeighbour<int> neighbours = null;
			if(cuboidMap != null){
				// if non-mirror then use non-mirrored KD-tree other wise use mirrored KD-ree
				if(GetCurrentMirrorAxis() == Axis.None){
					if(cuboidMap.KDTree == null){
						cuboidMap.RebuildKDTree(GetCurrentMirrorAxis());
					}
					// transform into local space then use KD tree to get the index
					Vector3 pointLocal = point.GetRelativePositionTo(cuboidMap.Pivot);
					neighbours = cuboidMap.KDTree.NearestNeighbors(new double[3]{pointLocal.x, pointLocal.y, pointLocal.z}, K);
				}
				else{
					if(cuboidMap.KDTree_mirror == null){
						cuboidMap.RebuildKDTree(GetCurrentMirrorAxis());
					}
					// transform into local space then use KD tree to get the index
					Vector3 pointLocal = point.GetRelativePositionTo(cuboidMap.Pivot);
					neighbours = cuboidMap.KDTree_mirror.NearestNeighbors(new double[3]{pointLocal.x, pointLocal.y, pointLocal.z}, K);
				}
				
				int[] kIndices = new int[K];
				float[] kDistances = new float[K];
				int k = 0;
				foreach(int neighbour in neighbours) {
					kIndices[k] = neighbour;
					kDistances[k] = (point-cuboidMap.References[neighbour]).sqrMagnitude;
					k++;
                }
				float[] blendWeights = new float[K];
				float maxDistance = kDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					blendWeights[i] = maxDistance + voxelDignalDistance/50f - kDistances[i];
				}
				//surfaceDistances.Print();
				Vector3 blendedPoint = Vector3.zero;
				Vector3 blendedGradient = Vector3.zero;
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					dis += blendWeights[i]/blendWeights.Sum() * realDistance[index];
					blendedPoint += blendWeights[i]/blendWeights.Sum() * cuboidMap.References[index];
					blendedGradient += blendWeights[i]/blendWeights.Sum() * GetWorldGradient(index); // here gradient need to mirrored as well
				}
				direction = blendedGradient.normalized;
			}
			return dis;
		}
		// ****************************************************************************************************************
		// *********************************************KNN search with KD-TREE********************************************
		// ****************************************************************************************************************



		/// <summary>
		/// return the indices for K smallest values in order
		/// </summary>
		/// <param name="array"></param>
		/// <param name="K"></param>
		/// <returns></returns>
		public static int[] FindKNNIndices(float[] array, int K){
			int[] kIndex = new int[K];
			float[] kValue = new float[K];
			for(int i=0; i<K; i++){
				kIndex[i] = int.MaxValue;
				kValue[i] = float.MaxValue;
			}
			for(int i=0; i<array.Length; i++){
				float dist = array[i];
				if(dist<kValue[K-1]){
					// if the value is small than Kth samllest value
					for(int j=0; j<K; j++){
						if(dist<kValue[j]){
							// first push
							for(int right=K-1; right>j; right--){
								kValue[right] = kValue[right-1];
								kIndex[right] = kIndex[right-1];
							}
							// then replace 
							kValue[j] = dist;
							kIndex[j] = i;
							// stop
							break;
						}
					}
				}
			}
			return kIndex;
		}

		/// <summary>
		///  this return KNN distance, but also filtered by the value, the smaller value the better
		/// </summary>
		/// <param name="array"></param>
		/// <param name="realValue"></param>
		/// <param name="K"></param>
		/// <returns></returns>
		public static int[] FindKNNSmallIndices(float[] array, float[] realValue, int K, float tolerent){
			int[] kIndex = new int[K];
			float[] kDistance = new float[K];
			float[] kValue = new float[K];
			for(int i=0; i<K; i++){
				kIndex[i] = int.MaxValue;
				kDistance[i] = float.MaxValue;
				kValue[i] = float.MaxValue;
			}
			float minDistance = array.Min();
			for(int i=0; i<array.Length; i++){
				float dist = array[i];
				float value = realValue[i];
				if((dist-minDistance<=tolerent && value<kValue[K-1])){
					for(int j=0; j<K; j++){
						if((value<kValue[j])){
							// first push
							for(int right=K-1; right>j; right--){
								kDistance[right] = kDistance[right-1];
								kIndex[right] = kIndex[right-1];
								kValue[right] = kValue[right-1];
							}
							// then replace 
							kDistance[j] = dist;
							kIndex[j] = i;
							kValue[j] = value;
							// stop
							break;
						}
					}
				}
			}
			return kIndex;
		}


		// ****************************************************************************************************************
		// *************************************KNN search with Local Index (new search)***********************************
		// ****************************************************************************************************************
		public float GetSignedDistanceValueViaLocalIndex(out Vector3 direction, Vector3 point, int K = 10){
			float dis = 0;
			direction = Vector3.zero;
			if(cuboidMap != null){
				int[] kIndices = new int[K];
				float[] kDistances = new float[K];
				{// get kIndices
					Vector3 boundSize = GetBounds().size;
					kIndices = FindKNNVoxelsViaNewSearch(point, GetCornerTrans(cuboidMap.Pivot, boundSize), boundSize, dim, K);
				}
				{// get kSqrDistance
					int k = 0;
					foreach(int index in kIndices) {
						kDistances[k] = (point-cuboidMap.References[index]).sqrMagnitude;
						k++;
					}
				}

				float[] blendWeights = new float[K];
				float maxDistance = kDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					blendWeights[i] = maxDistance + voxelDignalDistance/50f - kDistances[i];
				}
				//surfaceDistances.Print();
				Vector3 blendedPoint = Vector3.zero;
				Vector3 blendedGradient = Vector3.zero;
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					dis += blendWeights[i]/blendWeights.Sum() * realDistance[index];
					blendedPoint += blendWeights[i]/blendWeights.Sum() * cuboidMap.References[index];
					blendedGradient += blendWeights[i]/blendWeights.Sum() * GetWorldGradient(index); // here gradient need to mirrored as well
				}
				{
					// get surface point 
					Vector3 surfacePoint = blendedPoint - blendedGradient.normalized * dis;
					// then get the direction from input point to the surfacepoint
					direction = surfacePoint - point;
					// then get the distance from input point to the surfacepoint, have to use the following way to give a signed value
					dis = dis+(point-blendedPoint).magnitude;
				}
			}
			return dis;
		}
		// given an point, return sdf and norm direction of that point
		public float GetSignedValueAndNormDirectionViaLocalIndex(out Vector3 direction, Vector3 point, int K = 10){
			float dis = 0;
			direction = Vector3.zero;
			if(cuboidMap != null){
				int[] kIndices = new int[K];
				float[] kDistances = new float[K];
				{// get kIndices
					Vector3 boundSize = GetBounds().size;
					kIndices = FindKNNVoxelsViaNewSearch(point, GetCornerTrans(cuboidMap.Pivot, boundSize), boundSize, dim, K);
				}
				{// get kSqrDistance
					int k = 0;
					foreach(int index in kIndices) {
						kDistances[k] = (point-cuboidMap.References[index]).sqrMagnitude;
						k++;
					}
				}

				
				float[] blendWeights = new float[K];
				float maxDistance = kDistances.Max();
				float voxelDignalDistance = cuboidMap.GetStep().magnitude;
				for(int i=0; i<K; i++){
					// here add a small positive value which avoids K=1 and get 0 blend weights
					blendWeights[i] = maxDistance + voxelDignalDistance/50f - kDistances[i];
				}
				//surfaceDistances.Print();
				Vector3 blendedPoint = Vector3.zero;
				Vector3 blendedGradient = Vector3.zero;
				for(int i=0; i<K; i++){
					int index = kIndices[i];
					dis += blendWeights[i]/blendWeights.Sum() * realDistance[index];
					blendedPoint += blendWeights[i]/blendWeights.Sum() * cuboidMap.References[index];
					blendedGradient += blendWeights[i]/blendWeights.Sum() * GetWorldGradient(index); // here gradient need to mirrored as well
				}
				direction = blendedGradient.normalized;
			}
			return dis;
		}
		// ****************************************************************************************************************
		// *************************************KNN search with Local Index (new search)***********************************
		// ****************************************************************************************************************


		#if UNITY_EDITOR
		[CustomEditor(typeof(CheckSDF))]
		public class Interaction_Editor : Editor {
			public CheckSDF Target;
			void Awake() {
				Target = (CheckSDF)target;
			}
			public override void OnInspectorGUI() {
				Undo.RecordObject(Target, Target.name); // record the target such that it will be modified
				Utility.SetGUIColor(UltiDraw.Grey);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					Utility.ResetGUIColor();
					Target.slideMaxValue = EditorGUILayout.Slider(Target.slideMaxValue, 4f, -1f);
					Target.slideMinValue = EditorGUILayout.Slider(Target.slideMinValue, 4f, -1f);

					Target.dim = EditorGUILayout.Vector3IntField("Dimension", Target.dim);
					if(Utility.GUIButton("ExtractVolume", UltiDraw.DarkGrey, UltiDraw.White)) {
						Target.ExtractVoxel();
					}
					if(Utility.GUIButton("DrawPivot", Target.drawPivot ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
						Target.drawPivot = !Target.drawPivot;
					}
					if(Utility.GUIButton("DrawDistance", Target.drawDistance ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
						Target.drawDistance = !Target.drawDistance;
					}
					if(Utility.GUIButton("DrawGradient", Target.drawGradient ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
						Target.drawGradient = !Target.drawGradient;
					}
					if(Utility.GUIButton("DrawPointCloud", Target.drawPointCloud ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
						Target.drawPointCloud = !Target.drawPointCloud;
					}

					using(new EditorGUILayout.VerticalScope ("Box")) {
						Target.ball = (GameObject)EditorGUILayout.ObjectField("CurrentPosition" ,Target.ball, typeof(GameObject), true);
						Target.ball1 = (GameObject)EditorGUILayout.ObjectField("CurrentPosition1" ,Target.ball1, typeof(GameObject), true);
						Target.capsule = (GameObject)EditorGUILayout.ObjectField("CurrentCapsule" ,Target.capsule, typeof(GameObject), true);
						Target.K = EditorGUILayout.IntField("K-Value", Target.K);
						Target.DoubleOperation = EditorGUILayout.Toggle("DoubleOperation", Target.DoubleOperation);
						if(Utility.GUIButton("DrawTargetPoint", Target.drawTargetPoint ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
							Target.drawTargetPoint = !Target.drawTargetPoint;
						}
						if(Utility.GUIButton("DrawKNNVoxels", Target.drawKNNVoxels ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
							Target.drawKNNVoxels = !Target.drawKNNVoxels;
						}
						if(Utility.GUIButton("DrawKNNSurfacePoints", Target.drawKNNSurfacePoints ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
							Target.drawKNNSurfacePoints = !Target.drawKNNSurfacePoints;
						}
						if(Utility.GUIButton("DrawDeepestPoint", Target.drawDeepestPoint ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
							Target.drawDeepestPoint = !Target.drawDeepestPoint;
						}
					}
					using(new EditorGUILayout.VerticalScope ("Box")) {
						// here change the boundary just for hardcode checking
						if(Utility.GUIButton("MaxBounds", UltiDraw.Grey, UltiDraw.Black)) {
							Vector3 currentSize = Target.GetBounds().size;
							Vector3 newSize = Vector3.one * Mathf.Max(currentSize.x, currentSize.y, currentSize.z);
							Target.GetSDFBaker().SetBounds(newSize);

						}
						if(Utility.GUIButton("MeshBounds", UltiDraw.Grey, UltiDraw.Black)) {
							Target.GetSDFBaker().SetBoundsCenter(Target.GetMeshCenter()-Target.transform.GetWorldMatrix().GetPosition());
							Target.GetSDFBaker().SetBounds(Target.GetMeshSize());
						}
						
					}


					using(new EditorGUILayout.VerticalScope ("Box")) {
						if(Utility.GUIButton("DrawNNVoxelViaNewSearch", Target.drawNNVoxelViaNewSearch ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
							Target.drawNNVoxelViaNewSearch = !Target.drawNNVoxelViaNewSearch;
						}
						if(Utility.GUIButton("DrawKNNVoxelViaNewSearch", Target.drawKNNVoxelViaNewSearch ? UltiDraw.Green : UltiDraw.Grey, UltiDraw.Black)) {
							Target.drawKNNVoxelViaNewSearch = !Target.drawKNNVoxelViaNewSearch;
						}
					}
				}
				if(GUI.changed) {
					EditorUtility.SetDirty(Target);
				}
				serializedObject.ApplyModifiedProperties();
			}
		}
		#endif
	}
}