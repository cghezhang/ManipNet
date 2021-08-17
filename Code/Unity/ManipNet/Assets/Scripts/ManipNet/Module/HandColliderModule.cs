
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SDFr;


public class HandColliderModule
{
    public HandColliderBuilder[] handColliderBuilders;
    // public VoxelCollider[] voxelColliderBuilder;
    public ObjectColliderBuilder[] objectColliderBuilders;
    public bool DrawHandCapsule = false;
    public bool DrawContactInfo = false;
    public float frictionCoefficient = 0.5f;
    public bool DrawObjectCollider = true;
    public bool DrawObjectVelocity = false;
    public bool DrawObjectCenterOfMass = false;
    public Color handFreeColor = UltiDraw.Green.Transparent(0.5f);
    public Color handCollisionColor = UltiDraw.Red.Transparent(0.5f);
    public Color objFreeColor = UltiDraw.Grey.Transparent(0.5f);
    public Color objCollisionColor = UltiDraw.Cyan.Transparent(0.5f);

    public float capsuleRadiusScale = 1f;

    public int cuboidMapResolution = 10;


    // future distance calculation for geometry representation
    public bool calculateSenserDistance = true;
    public int pastDistanceLength = 5;
    public int futureDistanceLength = 5;
    public float posThreshold = 1.5f;
    public float negThreshold = -0.2f;
    public List<float[]> futureDistances = new List<float[]>();
    public bool drawFutureDistance = false;
    public int drawVetexIndex = 0;
    public bool drawProjection = false;
    public Color colorProjection = UltiDraw.Yellow;
    public float transProjection = 0.5f;
    public bool drawProjectionFree = false;
    public Color colorProjectionFree = UltiDraw.White;
    public float transProjectionFree= 0.5f;
    public bool drawNNDistance = false;
    public Color colorNNDistance = UltiDraw.Brown;
    public float transNNDistance = 0.5f;
    public bool drawNNDirection = false;
    public Color colorNNDirection = UltiDraw.Black;
    public float transNNDirection = 0.5f;

    public int futureDistanceZoomOut = 1;
    // this value is used to filter out the vertices on the skins, 0 gives 391 number, 1 gives 148 number
    public float filterHandDirectionValue = 0f; 
    public float filterWristValue = 0f;
    public float filterThumbDivisionValue = 0f;
    public float filterThumbDirectionValue = 0f;
    public int[] vertexIndices = new int[0];
    public Vector3[] vertexPositions = new Vector3[0];
    public Vector3[] vertexDirections = new Vector3[0];
    // this baked distance for the vertices position and direction
    public bool drawFullDistances = false;
    public MeshDistance[] bakedDistances = new MeshDistance[0];
    // past distance 
    public bool drawPastDistances = false;
    public List<float[]> pastDistances = new List<float[]>();
    // we also use future nearest distance from joint to the object
    public List<float[]> nnFutureDistances = new List<float[]>();
    public List<Vector3[]> nnFutureDirections = new List<Vector3[]>(); // directions from joints to the obj surface in global space
    public List<Vector3> nnNormDirections = new List<Vector3>(); // directions of current nearest points' norm direction 
    // for tip
    public bool drawTips = false;
    public bool drawTipVelocities = false;

	protected void DerivedDraw(){
        if(DrawHandCapsule){
            for(int iHand =0; iHand<handColliderBuilders.Length; iHand++){
                handColliderBuilders[iHand].DrawJointCapsules(handFreeColor, handCollisionColor);
            }
        }
        if(DrawContactInfo){
            for(int iHand =0; iHand<handColliderBuilders.Length; iHand++){
                handColliderBuilders[iHand].DrawJointContactInfo(frictionCoefficient);
            }
        }
        if(DrawObjectCollider){
            for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
                objectColliderBuilders[iObj].DrawContact(objFreeColor);
            }
        }
        if(DrawObjectVelocity){
            // Vector3[] objectP = editor.GetCurrentFrame().GetObjectPositions(editor.Mirror);
            // Vector3[] objectV = editor.GetCurrentFrame().GetObjectVelocities(editor.Mirror, 1f/editor.TargetFramerate);
            // UltiDraw.Begin();
            // for(int i=0; i<objectP.Length; i++) {
			// 	UltiDraw.DrawArrow(
			// 		objectP[i],
			// 		objectP[i] + objectV[i],
			// 		0.75f,
			// 		0.2f,
			// 		0.5f,
			// 		UltiDraw.DarkGreen.Transparent(0.5f)
			// 	);
			// }
            // UltiDraw.End();            
        }
        if(DrawObjectCenterOfMass){
            UltiDraw.Begin();
            for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
                Vector3 com = objectColliderBuilders[iObj].GetGlobalCenterOfMass();
                UltiDraw.DrawSphere(com, Quaternion.identity, 0.1f, UltiDraw.Brown);
            }
            UltiDraw.End();
        }
        if(drawFutureDistance && futureDistances.Count>0){
            // if draw future distance of projection sensor, we here draw the sphere on the capsule colider
            UltiDraw.Begin();
            UltiDraw.DrawSphere(vertexPositions[drawVetexIndex], Quaternion.identity, 0.1f, UltiDraw.DarkGrey); 
            float dis = Utility.GetSignedDistanceViaRayCasting(vertexPositions[drawVetexIndex], vertexDirections[drawVetexIndex], posThreshold, negThreshold);
            Color color = dis<posThreshold? colorProjection.Transparent(transProjection):colorProjectionFree.Transparent(transProjectionFree);
            UltiDraw.DrawArrow(
                vertexPositions[drawVetexIndex],
                vertexPositions[drawVetexIndex] + vertexDirections[drawVetexIndex] * dis,
                0.85f, 0.02f, 0.07f, color
            );           
            UltiDraw.End();
        }
        if(drawProjection && calculateSenserDistance && futureDistances.Count>0){
            float[] currentDistance = futureDistances[0];    
            UltiDraw.Begin();
            for(int i=0; i<vertexPositions.Length; i++) {
                if(currentDistance[i]<posThreshold && currentDistance[i]>negThreshold){
                    UltiDraw.DrawArrow(
                        vertexPositions[i],
                        vertexPositions[i] + vertexDirections[i] * currentDistance[i],
                        0.85f, 0.02f, 0.07f, colorProjection.Transparent(transProjection)
                    );
                }
                if(drawProjectionFree && currentDistance[i] == posThreshold){
                    UltiDraw.DrawArrow(
                        vertexPositions[i],
                        vertexPositions[i] + vertexDirections[i] * currentDistance[i]/2,
                        0.85f, 0.02f, 0.07f, colorProjectionFree.Transparent(transProjectionFree)
                    );
                }
            }
            // if(drawVetexIndex<vertexPositions.Length){
            //     // draw special points
            //     UltiDraw.DrawArrow(
            //         vertexPositions[drawVetexIndex],
            //         vertexPositions[drawVetexIndex] + vertexDirections[drawVetexIndex] * currentDistance[drawVetexIndex],
            //         0.75f, 0.0075f, 0.05f, UltiDraw.Blue
            //     );
            // }
            UltiDraw.End();  
        }
    }
	
    
    public void UpdateContactofHandAndObjectVoxels(){
        for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
            objectColliderBuilders[iObj].UpdateCuboidMap();
        } 
        if(objectColliderBuilders.Length==1){
            // todo here hard code for one object only
            // here is for update the current contact situations including the contact point and norm
            for(int iHand=0; iHand<handColliderBuilders.Length; iHand++){
                int nJoint = handColliderBuilders[iHand].jointColliders.Length;
                for(int iJoint=0; iJoint<nJoint; iJoint++){
                    // get the joint colliders
                    int nCollider =  handColliderBuilders[iHand].jointColliders[iJoint].nColliders;
                    handColliderBuilders[iHand].jointColliders[iJoint].ResetContactInfo();
                    for(int iCollider=0; iCollider<nCollider; iCollider++){
                        Vector3 segmentStart = handColliderBuilders[iHand].jointColliders[iJoint].GetCapsuleStartPoint(iCollider);
                        Vector3 segmentEnd = handColliderBuilders[iHand].jointColliders[iJoint].GetCapsuleEndPoint(iCollider);
                        Vector3 center = (segmentStart+segmentEnd)/2f;
                        float realRadius = handColliderBuilders[iHand].jointColliders[iJoint].capsules[iCollider].radius;
                        bool isContact = false;
                        Vector3 pointOnObj = Utility.GetClosestPointsFromOverlapCapsuleOnObj(segmentStart, segmentEnd, realRadius,
                                                                                                LayerMask.GetMask(HandObjectMotionParser.InteractionLayerName),
                                                                                                out isContact, 
                                                                                                capsuleRadiusScale);
                        // as we estimate by overlap capsule and judge by the distance to capsule center, so the nearest point might not exactly same as the point on capsule
                        Vector3 pointOnCapsule= handColliderBuilders[iHand].GetRealJointColliders(iJoint)[iCollider].ClosestPoint(pointOnObj);
                        Vector3 contactNorm = Vector3.zero;
                        float dis = objectColliderBuilders[0].GetCheckSDF().GetSignedValueAndNormDirectionViaKDTree(out contactNorm, pointOnObj);
                        // float dis = objectColliderBuilders[0].GetCheckSDF().GetSignedValueAndNormDirectionViaLocalIndex(out contactNorm, pointOnObj);
                        handColliderBuilders[iHand].jointColliders[iJoint].capsules[iCollider].isContact = isContact;
                        if(isContact){
                            handColliderBuilders[iHand].jointColliders[iJoint].capsules[iCollider].contactOnObject = pointOnObj;
                            handColliderBuilders[iHand].jointColliders[iJoint].capsules[iCollider].contactOnCapsule = pointOnCapsule;
                            handColliderBuilders[iHand].jointColliders[iJoint].capsules[iCollider].contactNormOnObject = contactNorm;
                        }
                        
                    }
                }
            }
        }
        else{
            // Debug.Log("Hand Object contact is so far only implemented for one object");
        }
    }

    public void UpdateJointDistance(Actor hand, HandColliderBuilder handColliderBuilder){
        for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
            objectColliderBuilders[iObj].UpdateCuboidMap();
        } 
        if(handColliderBuilder.jointDistances == null){
            handColliderBuilder.jointDistances = new float[hand.Bones.Length];
        }
        else if(hand.Bones.Length != handColliderBuilder.jointDistances.Length){
            if(handColliderBuilder.jointDistances.Length==0){
                handColliderBuilder.jointDistances = new float[hand.Bones.Length];
            }
            else{
                Debug.Log("mis-matching joint number between" + hand.Bones.Length  + " " + handColliderBuilder.jointDistances.Length);
                return;
            }
        }
        // for(int iJoint=0; iJoint<hand.Bones.Length; iJoint++){
        //     float jointDis = float.MaxValue;
        //     Vector3 jointP = hand.Bones[iJoint].Transform.position;
        //     for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
        //         MeshCollider meshCollider = objectColliderBuilders[iObj].GetMeshCollider();
        //         Vector3 objP = meshCollider.ClosestPoint(jointP);
        //         float ds = (jointP-objP).sqrMagnitude;
        //         if(jointDis>ds){
        //             jointDis = ds;
        //         }
        //     }
        //     handColliderBuilder.jointDistances[iJoint] = Mathf.Sqrt(jointDis);
        // }
        // // calculate the virtual tip joints distance
        // Vector3[] tipJoints = handColliderBuilder.GetFingerTipPositions();
        // if(handColliderBuilder.tipDistances == null){
        //     handColliderBuilder.tipDistances = new float[tipJoints.Length];
        // }
        // if(tipJoints.Length != handColliderBuilder.tipDistances.Length){
        //     if(handColliderBuilder.tipDistances.Length==0){
        //         handColliderBuilder.tipDistances = new float[tipJoints.Length];
        //     }
        //     else{
        //         Debug.Log("mis-matching joint number between" + tipJoints.Length  + " " + handColliderBuilder.tipDistances.Length);
        //         return;
        //     }
        // }
        // for(int iJoint=0; iJoint<tipJoints.Length; iJoint++){
        //     float jointDis = float.MaxValue;
        //     Vector3 jointP = tipJoints[iJoint];
        //     for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
        //         MeshCollider meshCollider = objectColliderBuilders[iObj].GetMeshCollider();
        //         Vector3 objP = meshCollider.ClosestPoint(jointP);
        //         float ds = (jointP-objP).sqrMagnitude;
        //         if(jointDis>ds){
        //             jointDis = ds;
        //         }
        //     }
        //     handColliderBuilder.tipDistances[iJoint] = Mathf.Sqrt(jointDis);
        // }
        // /// <summary>
        // /// previously, we direcly use the convex meshCollider.getClosestPoint to caculate the distance, 
        // /// but some geometries now will have multiple convex meshes
        // /// so one way is get all of these convex meshes and calculate. let's see
        // /// </summary> 
        // Vector3 direction = Vector3.zero;
        // for(int iJoint=0; iJoint<hand.Bones.Length; iJoint++){
        //     float jointDis = float.MaxValue;
        //     Vector3 jointP = hand.Bones[iJoint].Transform.position;
        //     for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
        //         float ds = objectColliderBuilders[iObj].GetCheckSDF().GetSignedDistanceValueViaKDTree(out direction, jointP);
        //         if(jointDis>ds){
        //             jointDis = ds;
        //         }
        //     }
        //     handColliderBuilder.jointDistances[iJoint] = jointDis;
        // }
        // // calculate the virtual tip joints distance
        // Vector3[] tipJoints = handColliderBuilder.GetFingerTipPositions();
        // if(handColliderBuilder.tipDistances == null){
        //     handColliderBuilder.tipDistances = new float[tipJoints.Length];
        // }
        // if(tipJoints.Length != handColliderBuilder.tipDistances.Length){
        //     if(handColliderBuilder.tipDistances.Length==0){
        //         handColliderBuilder.tipDistances = new float[tipJoints.Length];
        //     }
        //     else{
        //         Debug.Log("mis-matching joint number between" + tipJoints.Length  + " " + handColliderBuilder.tipDistances.Length);
        //         return;
        //     }
        // }
        // for(int iJoint=0; iJoint<tipJoints.Length; iJoint++){
        //     float jointDis = float.MaxValue;
        //     Vector3 jointP = tipJoints[iJoint];
        //     for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
        //         float ds =  objectColliderBuilders[iObj].GetCheckSDF().GetSignedDistanceValueViaKDTree(out direction, jointP);
        //         if(jointDis>ds){
        //             jointDis = ds;
        //         }
        //     }
        //     handColliderBuilder.tipDistances[iJoint] = jointDis;
        // }

        /// <summary>
        /// previously, we direcly use the convex meshCollider.getClosestPoint to caculate the distance, 
        /// but some geometries now will have multiple convex meshes
        /// so one way is get all of these convex meshes and calculate. let's see
        /// </summary> 
        for(int iJoint=0; iJoint<hand.Bones.Length; iJoint++){
            float jointDis = float.MaxValue;
            Vector3 jointP = hand.Bones[iJoint].Transform.position;
            for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
                MeshCollider[] meshColliders = objectColliderBuilders[iObj].GetMeshColliders();
                for(int iCollider=0; iCollider<meshColliders.Length; iCollider++){
                    Vector3 objP = meshColliders[iCollider].ClosestPoint(jointP);
                    float ds = (jointP-objP).sqrMagnitude;
                    if(jointDis>ds){
                        jointDis = ds;
                    }
                }
            }
            handColliderBuilder.jointDistances[iJoint] = Mathf.Sqrt(jointDis);
        }
        // calculate the virtual tip joints distance
        Vector3[] tipJoints = handColliderBuilder.GetFingerTipPositions();
        if(handColliderBuilder.tipDistances == null){
            handColliderBuilder.tipDistances = new float[tipJoints.Length];
        }
        if(tipJoints.Length != handColliderBuilder.tipDistances.Length){
            if(handColliderBuilder.tipDistances.Length==0){
                handColliderBuilder.tipDistances = new float[tipJoints.Length];
            }
            else{
                Debug.Log("mis-matching joint number between" + tipJoints.Length  + " " + handColliderBuilder.tipDistances.Length);
                return;
            }
        }
        for(int iJoint=0; iJoint<tipJoints.Length; iJoint++){
            float jointDis = float.MaxValue;
            Vector3 jointP = tipJoints[iJoint];
            for(int iObj=0; iObj<objectColliderBuilders.Length; iObj++){
                MeshCollider[] meshColliders = objectColliderBuilders[iObj].GetMeshColliders();
                for(int iCollider=0; iCollider<meshColliders.Length; iCollider++){
                    Vector3 objP = meshColliders[iCollider].ClosestPoint(jointP);
                    float ds = (jointP-objP).sqrMagnitude;
                    if(jointDis>ds){
                        jointDis = ds;
                    }
                }
            }
            handColliderBuilder.tipDistances[iJoint] = Mathf.Sqrt(jointDis);
        }
    }

    public static int[] UpdateCurrentSkinVertexIndices(Mesh sharedMesh, float dotValue, float wristValue, float thumbDivisionValue, float thumbDotValue){
        List<int> indices = new List<int>();
        for(int i=0; i<sharedMesh.normals.Length; i++){
            if(Vector3.Dot(sharedMesh.normals[i], new Vector3(0, -1, 0)) > dotValue){
                // if the direction is inner direction
                if(sharedMesh.vertices[i].x > wristValue || sharedMesh.vertices[i].x < - wristValue){
                    // if the not near the wrist
                    if(sharedMesh.vertices[i].z < thumbDivisionValue){
                        // if not thumb
                        indices.Add(i);
                    }
                }
            }
            if(sharedMesh.vertices[i].z > thumbDivisionValue){
                // if thumb
                if(Vector3.Dot(sharedMesh.normals[i], new Vector3(-1, 0, 0)) > thumbDotValue &&  sharedMesh.vertices[i].x < - wristValue){
                    // if the directoin is inner direction
                    indices.Add(i);
                }
            }
        }
        // further remove 
        int[] removeIndices = new int[]{8,25,25,27,47,47,48,48,50,98};
        for(int i=0;i<removeIndices.Length;i++){
            indices.RemoveAt(removeIndices[i]);
        }
        return indices.ToArray();
    }
    

    public static List<float[]> SenseFutureDistance(Vector3[] currentPositions, Vector3[] currentDirections,
                                                    ObjectColliderBuilder objectColliderBuilder, 
                                                    Matrix4x4[] rightWristTrajectory, Matrix4x4[] objTrajectory, 
                                                    float posThreshold, float negThreshold){
        List<float[]> distances = new List<float[]>();
        int nPoints = currentPositions.Length;
        int nSteps = rightWristTrajectory.Length;
        for(int iStep=0; iStep<nSteps; iStep++){
            float[] currentDistances = new float[nPoints];
            objectColliderBuilder.GetObject().transform.position = objTrajectory[iStep].GetPosition();
            objectColliderBuilder.GetObject().transform.rotation = objTrajectory[iStep].GetRotation();
            for(int iPoint=0; iPoint<nPoints; iPoint++){
                // get the global point position and direction
                Vector3 pointPosition = currentPositions[iPoint].GetRelativePositionTo(rightWristTrajectory[0]).GetRelativePositionFrom(rightWristTrajectory[iStep]);
                Vector3 pointDirection = currentDirections[iPoint].GetRelativeDirectionTo(rightWristTrajectory[0]).GetRelativeDirectionFrom(rightWristTrajectory[iStep]);
                /// todo so far supposing meshCollider is convex
                currentDistances[iPoint] = Utility.GetSignedDistanceViaRayCasting(pointPosition, pointDirection, posThreshold, negThreshold);
            }
            distances.Add(currentDistances);
        }
        // place the object back
        objectColliderBuilder.GetObject().transform.position = objTrajectory[0].GetPosition();
        objectColliderBuilder.GetObject().transform.rotation = objTrajectory[0].GetRotation();
        return distances;
    }


    /**
    here return the distance as well as directions in the world space
    note the distance is clamped by the threshold but the direction is not normalized
    */
    public static List<float[]> SenseNNFutureDistance(out List<Vector3[]> directions, Vector3[] currentJointPositions, ObjectColliderBuilder objectColliderBuilder, Matrix4x4[] rightWristTrajectory, Matrix4x4[] objTrajectory, float upThreshold){
        List<float[]> distances = new List<float[]>();
        directions = new List<Vector3[]>();
        Vector3 direction = Vector3.zero;
        int nJoint = currentJointPositions.Length;
        int nSamples = rightWristTrajectory.Length;
        for(int iSample=0; iSample<nSamples; iSample++){
            float[] currentDistances = new float[nJoint];
            Vector3[] currentDirections = new Vector3[nJoint];
            objectColliderBuilder.GetObject().transform.position = objTrajectory[iSample].GetPosition();
            objectColliderBuilder.GetObject().transform.rotation = objTrajectory[iSample].GetRotation();
            objectColliderBuilder.UpdateCuboidMap();
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                // get the global joint position
                Vector3 jointPosition = currentJointPositions[iJoint].GetRelativePositionTo(rightWristTrajectory[0]).GetRelativePositionFrom(rightWristTrajectory[iSample]);
                float dis = objectColliderBuilder.GetCheckSDF().GetSignedDistanceValueViaKDTree(out direction, jointPosition);
                // float dis = objectColliderBuilder.GetCheckSDF().GetSignedDistanceValueViaLocalIndex(out direction, jointPosition);
                // here we clamp the distance value but not the direction
                currentDistances[iJoint] = dis>upThreshold? upThreshold:dis; 
                currentDirections[iJoint] = direction; 
            }
            distances.Add(currentDistances);
            directions.Add(currentDirections);
        }

        // place the object back
        objectColliderBuilder.GetObject().transform.position = objTrajectory[0].GetPosition();
        objectColliderBuilder.GetObject().transform.rotation = objTrajectory[0].GetRotation();
        objectColliderBuilder.UpdateCuboidMap();
        return distances;
    }

    /**
    here return the norm direction of NN surface point 
    */
    public static List<Vector3> SenseNNSurfaceNorms(Vector3[] jointPositions, Vector3[] jointToSurface, ObjectColliderBuilder objectColliderBuilder){
        List<Vector3> norms = new List<Vector3>();
        if(jointPositions.Length != jointToSurface.Length){
            Debug.LogError("nums are not matching");
        }
        else{
            int nJoint = jointPositions.Length;
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                // get the global joint position
                Vector3 surfacePoint = jointPositions[iJoint] + jointToSurface[iJoint];
                Vector3 direction = new Vector3();
                float dis = objectColliderBuilder.GetCheckSDF().GetSignedValueAndNormDirectionViaKDTree(out direction, surfacePoint);
                // float dis = objectColliderBuilder.GetCheckSDF().GetSignedValueAndNormDirectionViaLocalIndex(out direction, surfacePoint);
                norms.Add(direction);
            }
        }
        return norms;
    }


    [System.Serializable]
	public class MeshDistance {
		/// <summary>
		/// for storing the contact from the ray-casting for mesh vertices
		/// </summary>
		public int nVertices;
        public float posThreshold;
        public float negThreshold;
        public float[] leftContacts;
        public float[] rightContacts;

		public MeshDistance(int in_nVertices, float in_posThreshold, float in_negThreshold){
            nVertices = in_nVertices;
            posThreshold = in_posThreshold;
            negThreshold = in_negThreshold;
            leftContacts = new float[nVertices];
            rightContacts = new float[nVertices];
		}
	}

}
