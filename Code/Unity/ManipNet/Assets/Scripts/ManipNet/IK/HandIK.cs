using SDFr;
using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

public static class HandIK
{   
    public static int[] tipParentIndex = new int[] {4, 7, 10, 13, 16};

    public static UltimateIK.Model PostJointDistanceIK(Actor hand, HandColliderBuilder handColliderBuilder, 
                                               CheckSDF checkSDF,
                                               float[] contacts, float contactThreshold, float discountThreshold,
                                               UltimateIK.Model IK, 
                                               out List<Vector3> contactPoints,
                                               bool isMain = true,
                                               Matrix4x4[] RestPose = null){
        int[] ignoreJointIndex = new int[] {0, 1, 2, 5, 8, 11, 14};

        // constant hard code 10 hinge joints in order for right hand
        int[] thumbLimitIndex = new int[] {3,4};
        int[] otherLimitIndex = new int[] {6,7, 9,10, 12,13, 15,16};

        //     // thumb joints
        // float[] thumbLimit1 = new float[] {0f, 73.75493f};
        // float[] thumbLimit2 = new float[] {-14.81322f, 62.29577f};
        //     // other joints
        // float[] otherLimit1 = new float[] {0f, 107.85714f};
        // float[] otherLimit2 = new float[] {0f, 82.142857f};
        // if(!isMain){
        //     thumbLimit1 = new float[] {-73.75493f, 0f};
        //     thumbLimit2 = new float[] {-62.29577f, 14.81322f};
        //     otherLimit1 = new float[] {-107.85714f, 0f};
        //     otherLimit2 = new float[] {-82.142857f, 0f};
        // }    
            // thumb joints 
        float[] thumbLimit1 = new float[] {0f-10f, 73.75493f+30f};
        float[] thumbLimit2 = new float[] {-14.81322f-10f, 62.29577f+30f};
            // other joints
        float[] otherLimit1 = new float[] {0f-10f, 107.85714f+30f};
        float[] otherLimit2 = new float[] {0f-10f, 82.142857f+30f};
        if(!isMain){
            thumbLimit1 = new float[] {-73.75493f-10f, 0f+30f};
            thumbLimit2 = new float[] {-62.29577f-10f, 14.81322f+30f};
            otherLimit1 = new float[] {-107.85714f-10f, 0f+30f};
            otherLimit2 = new float[] {-82.142857f-10f, 0f+30f};
        }
        List<float[]> thumbLimits = new List<float[]>();
        thumbLimits.Add(thumbLimit1);
        thumbLimits.Add(thumbLimit2);
        List<float[]> otherLimits = new List<float[]>();
        otherLimits.Add(otherLimit1);
        otherLimits.Add(otherLimit2);

        // initial contactPoints that want to output
        contactPoints = new List<Vector3>();
        int nTotalJoints = BinaryMocapParser.NUMBER_OF_FRAMES + tipParentIndex.Length; // here is the number of joints + tip joints
        if(contacts.Length != nTotalJoints){
            // we predict all the collider contact though only use the digit fingers, here is for checking, we hope to remove later.
            Debug.LogError("During IK : the contact number does not match collider number :" + contacts.Length + " "+ nTotalJoints);
        }
        else if(checkSDF == null){
            Debug.LogError("During IK : the checkSDF is null");
        }
        else{
            // initial all targets 
            Matrix4x4[] targets = new Matrix4x4[nTotalJoints];
            for(int iJoint=0; iJoint<nTotalJoints; iJoint++){
                targets[iJoint] = Matrix4x4.identity;
            }
            // will move if there is contact happen
            Transform[] currentTrans;
            if(IK==null || IK.Bones.Length==0){ // need to build IK
                // add mesh bone    
                hand.AddMeshBones(tipParentIndex, handColliderBuilder.GetFingerTipPositions());
                currentTrans = hand.GetOriginAndMeshBoneTransforms(tipParentIndex);
                Matrix4x4[] bakedCurrent = null;
                if(RestPose != null){
                    // if restPose is not null, then we build IK with rest pose and later will add joint limits
                    // first bake current Trans
                    bakedCurrent = new Matrix4x4[currentTrans.Length];
                    for(int iJoint=0; iJoint<currentTrans.Length; iJoint++){
                        bakedCurrent[iJoint] = Matrix4x4.TRS(currentTrans[iJoint].position, currentTrans[iJoint].rotation, Vector3.one);
                    }
                    // then need to set current as restPose. later will set back
                    for(int iJoint=0; iJoint<RestPose.Length; iJoint++){
                        currentTrans[iJoint].position = RestPose[iJoint].GetPosition();
                        currentTrans[iJoint].rotation = RestPose[iJoint].GetRotation();
                    }
                }
                // build the objectives
                IK = UltimateIK.BuildModel(IK, hand.Bones[0].Transform, currentTrans);
                IK.Activation = UltimateIK.ACTIVATION.Linear;

                if(RestPose != null){
                    // set the current Trans back
                    for(int iJoint=0; iJoint<currentTrans.Length; iJoint++){
                        currentTrans[iJoint].position = bakedCurrent[iJoint].GetPosition();
                        currentTrans[iJoint].rotation = bakedCurrent[iJoint].GetRotation();
                    }
                }
                Debug.Log("build IK with " + currentTrans.Length + "joints");
            }
            else{
                currentTrans = hand.GetOriginAndMeshBoneTransforms(tipParentIndex);
            }
            // get the joint positions
            Vector3[] jointPositions = hand.GetOriginAndMeshBonePositions(tipParentIndex);
            if(jointPositions.Length != nTotalJoints){
                Debug.LogError("During IK : the contact number does not match joint number :" + jointPositions.Length + " "+ nTotalJoints);
                return IK;
            }
            // set configures for IK
            for(int iIK=0; iIK<IK.Bones.Length; iIK++){
                IK.Bones[iIK].Active = true;
                // then set hinge joint
                if(RestPose != null){
                    if(thumbLimitIndex.Contains(iIK)){
                        int limitIndex = iIK-3;
                        IK.Bones[iIK].Joint = UltimateIK.JOINT.HingeZ;
                        IK.Bones[iIK].LowerLimit = thumbLimits[limitIndex][0];
                        IK.Bones[iIK].UpperLimit = thumbLimits[limitIndex][1];
                    }
                    else if(otherLimitIndex.Contains(iIK)){
                        int limitIndex = iIK%3==0? 0:1;
                        IK.Bones[iIK].Joint = UltimateIK.JOINT.HingeZ;
                        IK.Bones[iIK].LowerLimit = otherLimits[limitIndex][0];
                        IK.Bones[iIK].UpperLimit = otherLimits[limitIndex][1];
                    }
                }
            }
            for(int iObjective=0; iObjective<IK.Objectives.Length; iObjective++){
                // intialize with false in defualt, later will set the SolvePosition as true
                IK.Objectives[iObjective].SolvePosition = false;
                IK.Objectives[iObjective].SolveRotation = false;
            }
            for(int iJoint=0; iJoint<nTotalJoints; iJoint++){
                if(!ignoreJointIndex.Contains(iJoint)){
                    float predictedDistance = contacts[iJoint];
                    Vector3 currentJointPosition = jointPositions[iJoint];
                    Vector3 targetPosition = currentJointPosition;
                    if(predictedDistance<0.8f){
                        float capsuleRadius = 0f;
                        if(iJoint<nTotalJoints-5){
                            capsuleRadius = handColliderBuilder.jointColliders[iJoint].capsules[0].radius;
                            predictedDistance = predictedDistance * contactThreshold + capsuleRadius;
                        }
                        else{
                            predictedDistance = predictedDistance * contactThreshold;
                        }
                        int[] indices = new int[0];
                        float signedDistance = 0f;
                        // Vector3 surfacePoint = checkSDF.FindPointViaKNNSDF_new(currentJointPosition, out indices, out signedDistance, 20, 0.6f, -0.6f);
                        // Vector3 surfaceDir = checkSDF.GetNormDirectionViaKNN(surfacePoint, 20, 0.6f, -0.6f);
                        Vector3 surfacePoint = checkSDF.FindPointViaKNNSDF_new(currentJointPosition, out indices, out signedDistance, 30, 0.6f, -0.6f);
                        Vector3 surfaceDir = checkSDF.GetNormDirectionViaKNN(surfacePoint, 30, 0.6f, -0.6f);
                        float IKDiscount = discountThreshold;
                        IKDiscount = Mathf.Clamp(IKDiscount, 0f, 1f);
                        if(signedDistance<contactThreshold*2f){ // if real distance is winthin a threshold, because sometimes seems far away, but the distance is low? why?
                            if(signedDistance < capsuleRadius){ 
                                // Debug.Log("joint index" + iJoint);
                                // Debug.Log("signed distance " + signedDistance);
                                // Debug.Log("real distance " + (currentJointPosition-surfacePoint).magnitude);
                                // if smaller than radius, then we say penetration happens
                                // then we blend predicted distance with capsuleRadius
                                predictedDistance = capsuleRadius;
                                targetPosition = surfaceDir * predictedDistance + surfacePoint;
                                IK.Objectives[iJoint].Weight = 0.8f;
                            }
                            else{
                                // otherwise it is out side of the object, 
                                // then we blend predicted distance with signedDistance
                                predictedDistance = IKDiscount*predictedDistance+(1f-IKDiscount)*signedDistance;
                                targetPosition = (currentJointPosition - surfacePoint).normalized * predictedDistance + surfacePoint;
                                IK.Objectives[iJoint].Weight = 0.5f;
                            }
                            if(iJoint>=17){
                                contactPoints.Add(currentJointPosition);
                                contactPoints.Add(surfacePoint);
                            }
                            
                            // update current meshBone and targets
                            targets[iJoint] = Matrix4x4.TRS(targetPosition, currentTrans[iJoint].rotation, currentTrans[iJoint].lossyScale);
                            IK.Objectives[iJoint].SolvePosition = true;
                        }
                    }
                }
            }
            // we don't allow the root rotation and position as they are given 
            IK.Bones[0].Active = false;
            IK.RootTranslationY = false;
            IK.RootTranslationX = false;
            IK.RootTranslationZ = false;
            // here run the solve
            IK.Solve(targets);
        }
        //bettr return, otherwise need to rebuild everyframe/
        return IK;
    }

    
    public static UltimateIK.Model PostPeneIK(Actor hand, HandColliderBuilder handColliderBuilder, 
                                                     MeshCollider[] meshColliders,
                                                     float discountThreshold,
                                                     UltimateIK.Model IK, 
                                                     bool isMain = true,
                                                     Matrix4x4[] RestPose = null){
        int nMesh = meshColliders.Length;
        if(nMesh==0){
            Debug.Log("no mesh colliders");
            return IK;
        }
        // constant hard code 10 hinge joints in order for right hand
        int[] thumbLimitIndex = new int[] {3,4};
        int[] otherLimitIndex = new int[] {6,7, 9,10, 12,13, 15,16};
            // thumb joints 
        float[] thumbLimit1 = new float[] {0f-10f, 73.75493f+30f};
        float[] thumbLimit2 = new float[] {-14.81322f-10f, 62.29577f+30f};
            // other joints
        float[] otherLimit1 = new float[] {0f-10f, 107.85714f+30f};
        float[] otherLimit2 = new float[] {0f-10f, 82.142857f+30f};
        if(!isMain){
            thumbLimit1 = new float[] {-73.75493f-10f, 0f+30f};
            thumbLimit2 = new float[] {-62.29577f-10f, 14.81322f+30f};
            otherLimit1 = new float[] {-107.85714f-10f, 0f+30f};
            otherLimit2 = new float[] {-82.142857f-10f, 0f+30f};
        }
        List<float[]> thumbLimits = new List<float[]>();
        thumbLimits.Add(thumbLimit1);
        thumbLimits.Add(thumbLimit2);
        List<float[]> otherLimits = new List<float[]>();
        otherLimits.Add(otherLimit1);
        otherLimits.Add(otherLimit2);

        // init joint indices
        int[] applyJointIndex = ArrayExtensions.Concat(thumbLimitIndex, otherLimitIndex);
        int[] applyTipIndex = new int[] {17, 18, 19, 20, 21};

        // initial contactPoints that want to output
        int nTotalJoints = BinaryMocapParser.NUMBER_OF_FRAMES + tipParentIndex.Length; // here is the number of joints + tip joints

        {
            // initial all targets 
            Matrix4x4[] targets = new Matrix4x4[nTotalJoints];
            for(int iJoint=0; iJoint<nTotalJoints; iJoint++){
                targets[iJoint] = Matrix4x4.identity;
            }
            // will move if there is contact happen
            Transform[] currentTrans;
            if(IK==null || IK.Bones.Length==0){ // need to build IK
                // add mesh bone    
                hand.AddMeshBones(tipParentIndex, handColliderBuilder.GetFingerTipPositions());
                currentTrans = hand.GetOriginAndMeshBoneTransforms(tipParentIndex);
                Matrix4x4[] bakedCurrent = null;
                if(RestPose != null){
                    // if restPose is not null, then we build IK with rest pose and later will add joint limits
                    // first bake current Trans
                    bakedCurrent = new Matrix4x4[currentTrans.Length];
                    for(int iJoint=0; iJoint<currentTrans.Length; iJoint++){
                        bakedCurrent[iJoint] = Matrix4x4.TRS(currentTrans[iJoint].position, currentTrans[iJoint].rotation, Vector3.one);
                    }
                    // then need to set current as restPose. later will set back
                    for(int iJoint=0; iJoint<RestPose.Length; iJoint++){
                        currentTrans[iJoint].position = RestPose[iJoint].GetPosition();
                        currentTrans[iJoint].rotation = RestPose[iJoint].GetRotation();
                    }
                }
                // build the objectives
                IK = UltimateIK.BuildModel(IK, hand.Bones[0].Transform, currentTrans);
                IK.Activation = UltimateIK.ACTIVATION.Linear;

                if(RestPose != null){
                    // set the current Trans back
                    for(int iJoint=0; iJoint<currentTrans.Length; iJoint++){
                        currentTrans[iJoint].position = bakedCurrent[iJoint].GetPosition();
                        currentTrans[iJoint].rotation = bakedCurrent[iJoint].GetRotation();
                    }
                }
                Debug.Log("build IK with " + currentTrans.Length + "joints");
            }
            else{
                currentTrans = hand.GetOriginAndMeshBoneTransforms(tipParentIndex);
            }
            // get the joint positions
            Vector3[] jointPositions = hand.GetOriginAndMeshBonePositions(tipParentIndex);
            if(jointPositions.Length != nTotalJoints){
                Debug.LogError("During IK : the contact number does not match joint number :" + jointPositions.Length + " "+ nTotalJoints);
                return IK;
            }
            // set configures for IK
            for(int iIK=0; iIK<IK.Bones.Length; iIK++){
                IK.Bones[iIK].Active = true;
                // then set hinge joint
                if(RestPose != null){
                    if(thumbLimitIndex.Contains(iIK)){
                        int limitIndex = iIK-3;
                        IK.Bones[iIK].Joint = UltimateIK.JOINT.HingeZ;
                        IK.Bones[iIK].LowerLimit = thumbLimits[limitIndex][0];
                        IK.Bones[iIK].UpperLimit = thumbLimits[limitIndex][1];
                    }
                    else if(otherLimitIndex.Contains(iIK)){
                        int limitIndex = iIK%3==0? 0:1;
                        IK.Bones[iIK].Joint = UltimateIK.JOINT.HingeZ;
                        IK.Bones[iIK].LowerLimit = otherLimits[limitIndex][0];
                        IK.Bones[iIK].UpperLimit = otherLimits[limitIndex][1];
                    }
                }
            }
            for(int iObjective=0; iObjective<IK.Objectives.Length; iObjective++){
                // intialize with false in defualt, later will set the SolvePosition as true
                IK.Objectives[iObjective].SolvePosition = false;
                IK.Objectives[iObjective].SolveRotation = false;
            }

            for(int iJoint=0; iJoint<applyJointIndex.Length; iJoint++){
                // detect the collision by Physics.ComputePenetration
                CapsuleCollider currentCapsule1 = handColliderBuilder.GetRealJointColliders(applyJointIndex[iJoint])[0];
                CapsuleCollider currentCapsule2 = handColliderBuilder.GetRealJointColliders(applyJointIndex[iJoint]-1)[0];
                Vector3 finalDirection = Vector3.zero;
                float finalDistance = 0f;

                for(int iMesh=0; iMesh<nMesh; iMesh++){
                    MeshCollider mesh = meshColliders[iMesh];
                    
                    Vector3 tempDirection1 = Vector3.zero;
                    float tempDistance1 = 0f;
                    Vector3 tempDirection2 = Vector3.zero;
                    float tempDistance2 = 0f;
                    
                    bool overlapped1 = Physics.ComputePenetration(
                        mesh, mesh.gameObject.transform.position, mesh.gameObject.transform.rotation,
                        currentCapsule1, currentCapsule1.gameObject.transform.position, currentCapsule1.gameObject.transform.rotation,
                        out tempDirection1, out tempDistance1
                    );

                    bool overlapped2 = Physics.ComputePenetration(
                        mesh, mesh.gameObject.transform.position, mesh.gameObject.transform.rotation,
                        currentCapsule2, currentCapsule2.gameObject.transform.position, currentCapsule2.gameObject.transform.rotation,
                        out tempDirection2, out tempDistance2
                    );

                    if(overlapped1 && tempDistance1>finalDistance){
                        finalDistance = tempDistance1;
                        finalDirection = tempDirection1;
                    }
                    if(overlapped2 && tempDistance2>finalDistance){
                        finalDistance = tempDistance2;
                        finalDirection = tempDirection2;
                    }
                }
                if(finalDistance>0){
                    // if collider
                    Vector3 currentJoint = handColliderBuilder.jointColliders[applyJointIndex[iJoint]].GetCapsuleStartPoint(0);
                    Vector3 targetPosition;
                    targetPosition = currentJoint - finalDirection*finalDistance * 0.3f;
                    targets[applyJointIndex[iJoint]] = Matrix4x4.TRS(targetPosition, currentTrans[applyJointIndex[iJoint]].rotation, currentTrans[applyJointIndex[iJoint]].lossyScale);
                    IK.Objectives[applyJointIndex[iJoint]].SolvePosition = true;
                    IK.Objectives[applyJointIndex[iJoint]].Weight = 0.3f;
                }
            }
            
            for(int iJoint=0; iJoint<tipParentIndex.Length; iJoint++){
                // detect the collision by Physics.ComputePenetration
                CapsuleCollider currentCapsule1 = handColliderBuilder.GetRealJointColliders(tipParentIndex[iJoint])[0];
                Vector3 finalDirection = Vector3.zero;
                float finalDistance = 0f;
                for(int iMesh=0; iMesh<nMesh; iMesh++){
                    MeshCollider mesh = meshColliders[iMesh];
                    Vector3 tempDirection = Vector3.zero;
                    float tempDistance = 0f;
                    bool overlapped1 = Physics.ComputePenetration(
                        mesh, mesh.gameObject.transform.position, mesh.gameObject.transform.rotation,
                        currentCapsule1, currentCapsule1.gameObject.transform.position, currentCapsule1.gameObject.transform.rotation,
                        out tempDirection, out tempDistance
                    );
                    if(overlapped1 && tempDistance>finalDistance){
                        finalDistance = tempDistance;
                        finalDirection = tempDirection;
                    }
                }
                if(finalDistance>0){
                    // if collider
                    Vector3 currentJoint = handColliderBuilder.jointColliders[tipParentIndex[iJoint]].GetCapsuleEndPoint(0);
                    Vector3 targetPosition;
                    targetPosition = currentJoint - finalDistance*finalDirection * 0.3f;
                    targets[applyTipIndex[iJoint]] = Matrix4x4.TRS(targetPosition, currentTrans[tipParentIndex[iJoint]].rotation, currentTrans[tipParentIndex[iJoint]].lossyScale);
                    IK.Objectives[applyTipIndex[iJoint]].SolvePosition = true;
                    IK.Objectives[applyTipIndex[iJoint]].Weight = 0.3f;
                }
            }

            // fix the root 
            IK.Bones[0].Active = false;
            IK.RootTranslationY = false;
            IK.RootTranslationX = false;
            IK.RootTranslationZ = false;
            // here run the solve
            IK.Solve(targets);
        }
        //bettr return, otherwise need to rebuild everyframe/
        return IK;
    }
}
