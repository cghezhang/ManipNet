using System;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class HandColliderBuilder
{   
    public const int NUMBER_OF_FRAMES = 17; // it is indeed the number of bones
    public const int NUMBER_OF_DIGITS = 5;
    public const int RING_PROXIMAL = 12;
    public const int WRIST_FRAME = 1;
    public enum DigitIndex {DIGIT_THUMB, DIGIT_INDEX, DIGIT_MIDDLE, DIGIT_RING, DIGIT_PINKY};

    // special vertex indices
    public static int[] fingerTipIndices = new int[] {662, 601, 517, 430, 695};
    public static int[] fingerPadIndices = new int[] {644, 592, 508, 422, 686};
    public static int[] palmKnucklePoints = new int[] {207, 210, 220, 221, 223};
    public static int[] fingernailPoints = new int[] {663, 602, 518, 432, 696};
    public static int[] knuckleIndices = new int[] {301, 87, 83, 294, 144};
    public static int[,] palmColliderBaseIndices = new int[,] {{-1,-1}, {39, 264}, {33, 264}, {35, 706}, {36, 712}};
    public static int[] palmCenterIndex = new int[] {260};
    public static int[] correspondenceVertices = new int[] {32,  515, 425, 598, 658, 688, 281, 79,  190, 128,195, 375, 115, 467, 182, 542, 319, 204, 743, 628};

    // palmcapsules(first is null) and fingercapsules(first and second is null)
    public Capsule[] palmCapsules = new Capsule[NUMBER_OF_DIGITS];
    public Capsule[] fingerCapsules = new Capsule[NUMBER_OF_FRAMES];
    // joint colliders for building up colliders for each joint
    public JointCollider[] jointColliders = new JointCollider[NUMBER_OF_FRAMES];
    // real colliders (physical colliders)
    public Transform[] realJointColliders = new Transform[NUMBER_OF_FRAMES];
    // for each joint, we save the corresponding affecting mesh vertices
    public List<List<int>> jointVertexIndices = new List<List<int>>();

    // joint distance from nearest object surface
    public float[] jointDistances = new float[NUMBER_OF_FRAMES];
    // finger tips which is virtual joints
    public float[] tipDistances = new float[fingerPadIndices.Length];

    // for saving inner hands skinned mesh points
    public GameObject handSkinObj = null;

    public HandColliderBuilder(Actor actor, GameObject handSkin){
        Matrix4x4[] restTransformation = actor.GetBoneTransformations();
        if(restTransformation.Length == NUMBER_OF_FRAMES){         
            handSkinObj = handSkin;
            InitCapsuleColliders(restTransformation, handSkin.GetComponent<MeshFilter>().sharedMesh);
            InitJointColliders(restTransformation);
        }
        else{
            Debug.Log("Bone number " + restTransformation.Length +" does not match joint here " + NUMBER_OF_FRAMES);
        }
    }

    public HandColliderBuilder(Matrix4x4[] restTransformation, GameObject handSkin){
        if(restTransformation.Length == NUMBER_OF_FRAMES){    
            handSkinObj = handSkin;
            InitCapsuleColliders(restTransformation, handSkin.GetComponent<MeshFilter>().sharedMesh);
            InitJointColliders(restTransformation);
        }
        else{
            Debug.Log("Bone number " + restTransformation.Length +" does not match joint here " + NUMBER_OF_FRAMES);
        }
    }

    public int GetTotalColliderNumber(){
        int num = 0;
        for(int iJoint=0; iJoint<jointColliders.Length; iJoint++){
            num += jointColliders[iJoint].nColliders;
        }
        return num;
    }

    public CapsuleCollider[] GetRealJointColliders(int iJoint){
        CapsuleCollider[] colliders = new CapsuleCollider[0];
        if(realJointColliders[iJoint] ==null){
            Debug.LogError("no real colliders are initialized");
            return colliders;
        }
        else{
            colliders = realJointColliders[iJoint].gameObject.GetComponentsInChildren<CapsuleCollider>();
            return colliders;
        }
    }




    public float GetClosestDistance(){
        float dis = float.MaxValue;
        for(int i=0; i<jointDistances.Length; i++){
            dis = dis>jointDistances[i]? jointDistances[i]:dis;
        }
        for(int i=0; i<tipDistances.Length; i++){
            dis = dis>tipDistances[i]? tipDistances[i]:dis;
        }
        return dis;
    }
    public float[] GetJointDistances(){
        float[] jointDistances_ = new float[jointDistances.Length];
        for(int i=0; i<jointDistances.Length; i++){
            jointDistances_[i] = jointDistances[i];
        }
        return jointDistances_;
    }
    public float[] GetTipDistances(){
        float[] tipDistances_ = new float[tipDistances.Length];
        for(int i=0; i<tipDistances.Length; i++){
            tipDistances_[i] = tipDistances[i];
        }
        return tipDistances_;
    }

    // public float[] GetContactDistances(bool fromObjMesh){
    //     List<float> contactDistance = new List<float>();
    //     for(int iJoint=0; iJoint<jointColliders.Length; iJoint++){
    //         for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
    //             if(fromObjMesh){
    //                 contactDistance.Add(jointColliders[iJoint].capsules[iCollider].contactDistance);
    //             }
    //             else{
    //                 contactDistance.Add(jointColliders[iJoint].capsules[iCollider].contactDistance);
    //             }
    //         }
    //     }
    //     return contactDistance.ToArray();
    // }

    // public float[] GetContactLabel(float upThreshold, float downThreshold, bool fromObjMesh){
    //     List<float> contactLabel = new List<float>();
    //     for(int iJoint=0; iJoint<jointColliders.Length; iJoint++){
    //         for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
    //             float contactDistance;
    //             if(fromObjMesh){
    //                 contactDistance = jointColliders[iJoint].capsules[iCollider].contactDistance;
    //             }
    //             else{
    //                 contactDistance = jointColliders[iJoint].capsules[iCollider].contactDistance;
    //             }
    //             if(contactDistance>upThreshold){  
    //                 contactDistance = upThreshold;
    //             }
    //             contactLabel.Add(contactDistance/upThreshold); // normalize between 0 and 1
    //             // if(contactDistance<=downThreshold){  
    //             //     contactLabel.Add(1f);
    //             // }
    //             // else if(contactDistance>=upThreshold){
    //             //     contactLabel.Add(0f);
    //             // }
    //             // else{
    //             //     contactLabel.Add((upThreshold - contactDistance)/(upThreshold-downThreshold));
    //             // }
    //         }
    //     }
    //     return contactLabel.ToArray();
    // }

    public float[] GetJointContactLabel(float upThreshold, bool applyMeshOffset = false){
        // return distances from joint to the object
        // if applyMeshOffset, will delete an offset of the capsule radius
        List<float> contactLabel = new List<float>();
        float currentDistance = 0f;
        for(int iJoint=0; iJoint<jointDistances.Length; iJoint++){
            currentDistance = jointDistances[iJoint];
            if(applyMeshOffset && iJoint>1){
                currentDistance = currentDistance - jointColliders[iJoint].capsules[0].radius;
            }
            currentDistance = Mathf.Clamp(currentDistance, 0f, upThreshold);
            contactLabel.Add(currentDistance/upThreshold); // normalize between 0 to 1
        }
        for(int iJoint=0; iJoint<tipDistances.Length; iJoint++){
            // won't apply for the tip joints
            currentDistance = Mathf.Clamp(tipDistances[iJoint], 0f, upThreshold);
            contactLabel.Add(currentDistance/upThreshold);
        }
        return contactLabel.ToArray();
    }

    public Vector3[] GetFingerTipPositions(Axis mirrorAxis = Axis.None){
        // here hard code the finger tip joint index
        int[] fingerTipJointIndex = new int[]{4,7,10,13,16};
        int indexCap = 0; // only one capsule in these joints
        // here hard code the finger tip joint index
        int numTips = NUMBER_OF_DIGITS;
        Vector3[] fingerTipPositions = new Vector3[numTips];
        for(int iJoint=0; iJoint<numTips; iJoint++){
            fingerTipPositions[iJoint] = jointColliders[fingerTipJointIndex[iJoint]].GetCapsuleEndPoint(indexCap).GetMirror(mirrorAxis);
        }
        return fingerTipPositions;
    }
    public Vector3[] GetPalmCapsuleStartPositions(Axis mirrorAxis = Axis.None){
        // here hard code the finger tip joint index
        int palmJointIndex = 0;
        int numPalmCapsules = 4;
        // here hard code the finger tip joint index
        Vector3[] positions = new Vector3[numPalmCapsules];
        for(int iCap=0; iCap<numPalmCapsules; iCap++){
            positions[iCap] = jointColliders[palmJointIndex].GetCapsuleStartPoint(iCap).GetMirror(mirrorAxis);
        } 
        return  positions;
    }
    public Vector3[] GetPalmCapsuleEndPositions(Axis mirrorAxis = Axis.None){
        int palmJointIndex = 0;
        int numPalmCapsules = 4;
        Vector3[] positions = new Vector3[numPalmCapsules];
        for(int iCap=0; iCap<numPalmCapsules; iCap++){
            positions[iCap] = jointColliders[palmJointIndex].GetCapsuleEndPoint(iCap).GetMirror(mirrorAxis);
        } 
        return  positions;
    }

    public void UpdateJointColliderWorldT(Actor actor, bool updateRealColliders){
        Matrix4x4[] parentsT = actor.GetBoneTransformations();
        if(parentsT.Length == NUMBER_OF_FRAMES){
            for(int iJoint=0; iJoint<NUMBER_OF_FRAMES; iJoint++){
                for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
                    jointColliders[iJoint].worldTransformation[iCollider] = parentsT[iJoint] * jointColliders[iJoint].localTransformation[iCollider];
                }
                // here we in default update the real colliders, if there is no real collider, we will link to existing one or build from the actor
                if(updateRealColliders){
                    if(realJointColliders[iJoint]==null){
                     buildRealColliders(actor);
                    }
                    for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
                        realJointColliders[iJoint].GetChild(iCollider).position = (parentsT[iJoint] * jointColliders[iJoint].localTransformation[iCollider]).GetPosition();
                        realJointColliders[iJoint].GetChild(iCollider).rotation = (parentsT[iJoint] * jointColliders[iJoint].localTransformation[iCollider]).GetRotation();
                        // scale , here in default is 1
                        // lesson, here we need to modify the most child objects, because of the mirror problem, we can;t easily just update the joint!!
                    }
                }
            }    
        }
        else{Debug.LogError("length of parents transformation is wrong");}
    }

    
    public void buildRealColliders(Actor actor, LayerMask mask = default(LayerMask)){
        if(actor.Bones.Length == NUMBER_OF_FRAMES){
            Transform handObj = actor.GetRoot().parent;
            Transform handCollider =  handObj.Find(actor.name + "Collider");
            if(handCollider != null){
                for(int iJoint=0; iJoint<NUMBER_OF_FRAMES; iJoint++){
                    realJointColliders[iJoint] = handCollider.Find(actor.Bones[iJoint].GetName());
                    if(realJointColliders[iJoint] == null){
                        Debug.LogError("something wrong when initialize the real colliders");
                    }
                }
            }
            else{
                handCollider = new GameObject(actor.name + "Collider").transform;
                handCollider.parent = handObj;
                for(int iJoint=0; iJoint<NUMBER_OF_FRAMES; iJoint++){
                    Transform currentColliderJ = new GameObject(actor.Bones[iJoint].GetName()).transform;
                    currentColliderJ.parent = handCollider;
                    currentColliderJ.position = actor.Bones[iJoint].Transform.position;
                    currentColliderJ.rotation = actor.Bones[iJoint].Transform.rotation;
                    for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
                        Transform currentCollider = new GameObject(actor.Bones[iJoint].GetName()+"Collider").transform;
                        currentCollider.parent = currentColliderJ;
                        CapsuleCollider currentCapsuleCollider = currentCollider.gameObject.AddComponent<CapsuleCollider>();
                        
                        // capsule collider local transformation, these are not neccessary, but i just want to highlight
                        currentCapsuleCollider.center = Vector3.zero;
                        currentCapsuleCollider.direction = 1; // here let the height direction is the Y-axis, actually the default is already Y-axis 
                        currentCapsuleCollider.isTrigger = false; // don't need the trigger here
                        // redundantly set the layer mask
                        currentCollider.gameObject.layer = mask;

                        // here is important for setting the height and radius
                        currentCapsuleCollider.height = jointColliders[iJoint].capsules[iCollider].length;
                        currentCapsuleCollider.radius = jointColliders[iJoint].capsules[iCollider].radius;

                        // local position, later we only update the joint 
                        currentCollider.localPosition = jointColliders[iJoint].localTransformation[iCollider].GetPosition();
                        currentCollider.localRotation = jointColliders[iJoint].localTransformation[iCollider].GetRotation();
                        // no scale, because all should be 1   
                    }
                    realJointColliders[iJoint] = currentColliderJ;
                }
            }
        }
        else{
            Debug.LogError("actor bone number is not matching when build the real colliders");
        }
    }

    public void InitJointColliders(Matrix4x4[] restTransformation){
        for(int i=0; i<NUMBER_OF_FRAMES; i++){
            // wrist
            if(i==0){
                jointColliders[i] = new JointCollider(i, restTransformation[i], palmCapsules);
            }
            else{
                Capsule[] capsuleColliders_one = {fingerCapsules[i]};
                jointColliders[i] = new JointCollider(i, restTransformation[i], capsuleColliders_one);
            }
        }
    }
    public void InitCapsuleColliders(Matrix4x4[] restTransformation, Mesh mesh){
        if(restTransformation.Length != NUMBER_OF_FRAMES){
            Debug.LogError("We so far only deal with skeleton with :" + NUMBER_OF_FRAMES + "joints");
            return;
        }
        Vector3[] vertices = mesh.vertices;
        BoneWeight[] boneWeights = mesh.boneWeights;
        int nVertices = vertices.Length;
        if( nVertices!= boneWeights.Length){
            Debug.LogError("mesh is wrong");
        }

        // calculate palm collider characteristics
        Vector3[] palmColliderP = new Vector3[NUMBER_OF_DIGITS];
        Vector3[] palmColliderD = new Vector3[NUMBER_OF_DIGITS];
        // approximate the palm with 4 capsules --  corresponding to the Index to Pinky proximal joints,
        // skipping the thumb
        for(int iDigit =(int)DigitIndex.DIGIT_INDEX; iDigit<NUMBER_OF_DIGITS; iDigit++){
            int rootFrame = 2+3*iDigit;
            Vector3 palmColliderEndpoint = restTransformation[rootFrame].GetPosition();
            if(iDigit == (int)DigitIndex.DIGIT_PINKY){
                Vector3 ringProximalJoint = restTransformation[RING_PROXIMAL].GetPosition();
                palmColliderEndpoint = (palmColliderEndpoint + ringProximalJoint) * 0.5f;
            }
            palmColliderP[iDigit] = (vertices[palmColliderBaseIndices[iDigit,0]] + vertices[palmColliderBaseIndices[iDigit,1]]) * 0.5f;
            // point the collider at the endpoint
            palmColliderD[iDigit] = palmColliderEndpoint;
        }

        // for all vertices skinned to the palm,
        //  find the closest palm capsule center-line to each vertex,
        //  and add its contribution to the mean distance
        RunningStatistics[] meanDistPerPalmCollider = new RunningStatistics[NUMBER_OF_DIGITS];
        for(int i=0; i<NUMBER_OF_DIGITS; i++){
            meanDistPerPalmCollider[i] = new RunningStatistics();
        }
        for(int iVertex=0; iVertex<nVertices; iVertex++){
            // warning, WRIST_FRAME=1, which said actually the palm is skinned with index 1
            if(boneWeights[iVertex].boneIndex0 != WRIST_FRAME){
              continue;
            }
            Vector3 vertex = mesh.vertices[iVertex];
            float distSqrMin = 9999999;
            int digitClosest = -1;
            for(int iDigit =(int)DigitIndex.DIGIT_INDEX; iDigit<NUMBER_OF_DIGITS; iDigit++){
                Vector3 p = palmColliderP[iDigit];
                Vector3 d = palmColliderD[iDigit];
                
                Vector3 p_seg = Utility.ClosestPointOnLineSegment(p, d, vertex);
                float distSqr = (p_seg- vertex).sqrMagnitude;
                if (distSqr < distSqrMin) {
                    digitClosest = iDigit;
                    distSqrMin = distSqr;
                }
            }
            if (digitClosest >= 0) {
                meanDistPerPalmCollider[digitClosest].Add(Mathf.Sqrt(distSqrMin));
            }
        }
        // each finger has 3 segments, so 3 colliders
        for(int iDigit=0; iDigit<NUMBER_OF_DIGITS; iDigit++){
            Capsule prevCapsule = new Capsule();
            // proximal and intermediate capsules
            for (int j = 0; j < 2; j++) {
                int rootFrame = 2 + 3 * iDigit + j;
                Vector3 rootPoint = restTransformation[rootFrame].GetPosition();
                Vector3 endPoint = restTransformation[rootFrame+1].GetPosition();
                // Now look at every vertex and figure out what the bounds are on its distance
                // from the segment:  
                RunningStatistics meanDist = new RunningStatistics();
                for(int iVertex=0; iVertex<nVertices; iVertex++){
                    if(boneWeights[iVertex].boneIndex0 != rootFrame){
                        continue;
                    }
                    Vector3 vertex = mesh.vertices[iVertex];
                    Vector3 p_seg = Utility.ClosestPointOnLineSegment(rootPoint, endPoint, vertex);
                    meanDist.Add((p_seg-vertex).magnitude);
                }
                Capsule capsule = new Capsule();
                capsule.startP = rootPoint;
                capsule.endP = endPoint;
                capsule.radius = meanDist.Mean();
                capsule.length = (capsule.endP-capsule.startP).magnitude;
                fingerCapsules[rootFrame] = capsule;
                prevCapsule = capsule;
            }

            // distall capsule
            {
                int rootFrame = 2 + 3 * iDigit + 2;
                Vector3 rootPoint = restTransformation[rootFrame].GetPosition();
                Vector3 tipPoint = mesh.vertices[fingerTipIndices[iDigit]];
                // The tip capsule is special.  We'll reuse the radius from the previous frame 
                // and adjust it so that it just barely brushes the end of the finger:
                float tipPointDist = (tipPoint - rootPoint).magnitude;
                float tipRad = 0.95f * Mathf.Min(tipPointDist, prevCapsule.radius);
                Capsule capsule = new Capsule();
                capsule.startP = rootPoint;
                capsule.endP = tipPoint;
                capsule.radius = tipRad;
                capsule.length = (capsule.endP-capsule.startP).magnitude;
                fingerCapsules[rootFrame] = capsule;
            }

            // palm capsule-set contribution from this digit
            if(iDigit != (int)DigitIndex.DIGIT_THUMB){
                Capsule capsule = new Capsule();
                capsule.radius = meanDistPerPalmCollider[iDigit].Mean();
                capsule.startP = palmColliderP[iDigit];

                // shorten the capsule so that the tip just touches the knuckle
                Vector3 normal = (palmColliderD[iDigit]-palmColliderP[iDigit]).normalized;
                float capsuleLength = Mathf.Max((palmColliderD[iDigit]-palmColliderP[iDigit]).magnitude - capsule.radius, capsule.radius);
                capsule.endP = capsule.startP + normal*capsuleLength;
                capsule.length = (capsule.endP-capsule.startP).magnitude;
                palmCapsules[iDigit] = capsule; 
            }
        }
    }

    // Draw
    public void DrawJointCapsules(Color freeColor, Color collisionColor){
        // draw based in contact situation
        UltiDraw.Begin();
        for(int iJoint=0; iJoint<jointColliders.Length; iJoint++){
            for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
                // Debug.Log("draw " + iJoint +" " + iCollider + "  value " + jointColliders[iJoint].contactDistance[iCollider] );
                Color currentColor;
                currentColor = jointColliders[iJoint].capsules[iCollider].isContact? collisionColor:freeColor;
                // draw capsule
                UltiDraw.DrawCapsule(
                    jointColliders[iJoint].worldTransformation[iCollider].GetPosition(), jointColliders[iJoint].worldTransformation[iCollider].GetRotation(),
                    jointColliders[iJoint].capsules[iCollider].radius * 2, 
                    (jointColliders[iJoint].capsules[iCollider].endP - jointColliders[iJoint].capsules[iCollider].startP).magnitude,
                    currentColor
                );
            }
        }  
        UltiDraw.End();
    }
    public void DrawJointContactInfo(float mu){
        UltiDraw.Begin();
        for(int iJoint=0; iJoint<jointColliders.Length; iJoint++){
            for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
                
                {// draw contact point and norm
                    if(jointColliders[iJoint].capsules[iCollider].isContact){
                        UltiDraw.DrawSphere(jointColliders[iJoint].capsules[iCollider].contactOnObject, Quaternion.identity, 0.1f, UltiDraw.Purple);
                        UltiDraw.DrawSphere(jointColliders[iJoint].capsules[iCollider].contactOnCapsule, Quaternion.identity, 0.1f, UltiDraw.Blue);
                        UltiDraw.DrawArrow(
                            jointColliders[iJoint].capsules[iCollider].contactOnObject,
                            jointColliders[iJoint].capsules[iCollider].contactOnObject + jointColliders[iJoint].capsules[iCollider].contactNormOnObject,
                            0.75f, 0.02f, 0.05f, UltiDraw.Brown
                        );
                    }
                }
                {
                    //draw four basis for each force 
                    if(jointColliders[iJoint].capsules[iCollider].isContact){
                        Vector3[] basis = PhysicsUtility.Get4GlobalBasis(mu, jointColliders[iJoint].capsules[iCollider].contactNormOnObject);
                        for(int iBasis=0; iBasis<basis.Length; iBasis++){
                            UltiDraw.DrawArrow(
                                jointColliders[iJoint].capsules[iCollider].contactOnObject,
                                jointColliders[iJoint].capsules[iCollider].contactOnObject + basis[iBasis].normalized,
                                0.75f, 0.02f, 0.05f, UltiDraw.Cyan.Transparent(0.5f)
                            );
                        }
                    }
                }
            }
        }  
        UltiDraw.End();
    }

    public void DrawJointColliders(Color freeColor, Color collisionColor, float[] contactLabels, float threshold){
        UltiDraw.Begin();
        int currentColliderIndex = 0;
        for(int iJoint=0; iJoint<jointColliders.Length; iJoint++){
            for(int iCollider=0; iCollider<jointColliders[iJoint].nColliders; iCollider++){
                // Debug.Log("draw " + iJoint +" " + iCollider + "  value " + jointColliders[iJoint].contactDistance[iCollider] );
                Color currentColor = contactLabels[currentColliderIndex]<threshold ? collisionColor:freeColor;
                UltiDraw.DrawCapsule(
                    jointColliders[iJoint].worldTransformation[iCollider].GetPosition(), jointColliders[iJoint].worldTransformation[iCollider].GetRotation(),
                    jointColliders[iJoint].capsules[iCollider].radius * 2, 
                    (jointColliders[iJoint].capsules[iCollider].endP - jointColliders[iJoint].capsules[iCollider].startP).magnitude,
                    currentColor
                );
                currentColliderIndex++;
            }
        }  
        UltiDraw.End();
    }

    // Utility
    


    [System.Serializable]
    public class Capsule{
        public Vector3 startP;
        public Vector3 endP;
        public Vector3 center;
        public Quaternion rotation;
        public float length;
        public float radius;

        // === so far for physics evaluation only ===
        // one capsule at most only has one contact point
        // all features are in world space
        // the contact on the object and capsule should be the same, but the real estimation might have minnor difference
        public bool isContact; 
		public Vector3 contactOnObject; 
        public Vector3 contactOnCapsule; 
		public Vector3 contactNormOnObject; 
        // === so far for physics evaluation only ===
    }

    [System.Serializable]
    public class JointCollider{
        public int parentJoint;
        public int nColliders;
        // one joint may have multiple capsules (so far only the thumb)
        public Capsule[] capsules;
        // transformations are the position and rotation of the center point 
        // transformation relative to the parent joint 
        public Matrix4x4[] localTransformation;
        // transformation in the world space
        public Matrix4x4[] worldTransformation;

 

        public JointCollider(int in_parentJoint, Matrix4x4 restJointTransformation,  Capsule[] in_capsules) {
            parentJoint = in_parentJoint;
            if(in_capsules == null || in_capsules.Length == 0){
                nColliders = 0;
                Debug.Log("joint " + in_parentJoint+ " has no collider");
            }
            else{
                List<Capsule> capsulesList = new List<Capsule>();
                List<Matrix4x4> localTransformationList = new List<Matrix4x4>();
                for(int i=0; i<in_capsules.Length; i++){
                    if(in_capsules[i]!=null){
                        capsulesList.Add(in_capsules[i]);
                        Vector3 capsuleCenterPosition = in_capsules[i].startP * 0.5f + in_capsules[i].endP * 0.5f;
                        Quaternion capsulesCenterRotation = Quaternion.FromToRotation(Vector3.up, in_capsules[i].endP - in_capsules[i].startP) * Quaternion.identity;
                        Matrix4x4 capsulesTransformation = Matrix4x4.TRS(capsuleCenterPosition, capsulesCenterRotation, Vector3.one);
                        localTransformationList.Add(capsulesTransformation.GetRelativeTransformationTo(restJointTransformation));
                    }
                }
                nColliders = capsulesList.Count;
                capsules = capsulesList.ToArray();
                localTransformation = localTransformationList.ToArray();
                // initialize the world transformation
                worldTransformation = new Matrix4x4[localTransformation.Length];
                Array.Copy(localTransformation, worldTransformation, localTransformation.Length);
            }
        }
        public void ResetContactInfo(){
            for(int i=0; i<nColliders; i++){
                capsules[i].isContact = false;
                capsules[i].contactOnObject = Vector3.zero;
                capsules[i].contactOnCapsule = Vector3.zero;
                capsules[i].contactNormOnObject = Vector3.zero;
            }
        }
        public Vector3 GetCapsuleStartPoint(int capsuleIndex){
            if(capsuleIndex>=nColliders){
                Debug.LogError("out of boundary");
                return Vector3.zero;
            }
            float halfLength = (capsules[capsuleIndex].startP - capsules[capsuleIndex].endP).magnitude/2f;
            return worldTransformation[capsuleIndex].GetPosition() + worldTransformation[capsuleIndex].MultiplyVector(new Vector3(0f, -halfLength, 0f));
        }
        public Vector3 GetCapsuleEndPoint(int capsuleIndex){
            if(capsuleIndex>=nColliders){
                Debug.LogError("out of boundary");
                return Vector3.zero;
            }
            float halfLength = (capsules[capsuleIndex].startP - capsules[capsuleIndex].endP).magnitude/2f;
            return worldTransformation[capsuleIndex].GetPosition() + worldTransformation[capsuleIndex].MultiplyVector(new Vector3(0f, halfLength, 0f));
        }

    }
}
