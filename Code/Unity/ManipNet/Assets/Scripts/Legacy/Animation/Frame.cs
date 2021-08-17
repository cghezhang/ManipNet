
using UnityEngine;
using System;

[System.Serializable]
public class Frame {
	public const int dimAngularVelocity = 6;
	public const string wristName = "WRIST";

	public MotionData Data;
	public int Index;
	public float Timestamp;
	public double[] jointAngles;
	public Matrix4x4[] World;
	public Matrix4x4[] objectsWorld;
	public int numActors;
	public int numObjects;

	// add afterwards
	public Vector3[] tipWorldPositions;

	public Frame(MotionData data, int index, float timestamp, int in_numActors, int in_numObjects) {
		Data = data;
		Index = index;
		Timestamp = timestamp;
		numActors = in_numActors;
		numObjects = in_numObjects;
		// I flatten the hands transformations here
		jointAngles = new double[numActors * Data.Dofs.Sum()];
		World = new Matrix4x4[numActors * Data.Source.Bones.Length];
		objectsWorld = new Matrix4x4[numObjects];
	}


	public Frame GetFirstFrame() {
		return Data.Frames[0];
	}

	public Frame GetLastFrame() {
		return Data.Frames[Data.Frames.Length-1];
	}

	public float[] GetBoneMasses(bool mirrored) {
		float[] masses = new float[World.Length];
		for(int i=0; i<masses.Length; i++) {
			masses[i] = GetBoneMass(i, mirrored);
		}
		return masses;
	}

	public float[] GetBoneMasses(string[] bones, bool mirrored) {
		float[] masses = new float[bones.Length];
		for(int i=0; i<masses.Length; i++) {
			masses[i] = GetBoneMass(bones[i], mirrored);
		}
		return masses;
	}

	public float GetBoneMass(string bone, bool mirrored) {
		return GetBoneMass(Data.Source.FindBone(bone).Index, mirrored);
	}

	public float GetBoneMass(int index, bool mirrored) {
		return mirrored ? Data.Source.Bones[Data.Symmetry[index]].Mass : Data.Source.Bones[index].Mass;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//WORLD
	////////////////////////////////////////////////////////////////////////////////////////////////////
	public Matrix4x4[] GetBoneTransformations(bool mirrored) {
		Matrix4x4[] transformations = new Matrix4x4[World.Length];
		for(int i=0; i<World.Length; i++) {
			transformations[i] = GetBoneTransformation(i, mirrored);
		}
		return transformations;
	}

	public Matrix4x4[] GetBoneTransformations(string[] bones, bool mirrored) {
		Matrix4x4[] transformations = new Matrix4x4[bones.Length];
		for(int i=0; i<transformations.Length; i++) {
			transformations[i] = GetBoneTransformation(bones[i], mirrored);
		}
		return transformations;
	}

	public Matrix4x4 GetBoneTransformation(string bone, bool mirrored) {
		return GetBoneTransformation(Data.Source.FindBone(bone).Index, mirrored);
	}

	public Matrix4x4 GetBoneTransformation(int index, bool mirrored) {
		Matrix4x4 m = mirrored ? World[Data.Symmetry[index]].GetMirror(Data.MirrorAxis) : World[index];
		Vector3 o = mirrored ? Data.Offset.GetMirror(Data.MirrorAxis) : Data.Offset;
		m[0,3] = Data.Scale * (m[0,3] + o.x);
		m[1,3] = Data.Scale * (m[1,3] + o.y);
		m[2,3] = Data.Scale * (m[2,3] + o.z);
		return m;
		/*
		return 
			Matrix4x4.TRS(Data.Offset, Quaternion.identity, Data.Scaling * Vector3.one) * 
			(mirrored ? World[Data.Symmetry[index]].GetMirror(Data.MirrorAxis.GetAxis()) : World[index]) * 
			Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(Data.Source.Bones[index].Alignment), Vector3.one);
			*/
	}

	public Vector3[] GetBoneVelocities(bool mirrored, float delta) {
		Vector3[] velocities = new Vector3[Data.Source.Bones.Length];
		for(int i=0; i<World.Length; i++) {
			velocities[i] = GetBoneVelocity(i, mirrored, delta);
		}
		return velocities;
	}

	public Vector3[] GetBoneVelocities(string[] bones, bool mirrored, float delta) {
		Vector3[] velocities = new Vector3[bones.Length];
		for(int i=0; i<velocities.Length; i++) {
			velocities[i] = GetBoneVelocity(bones[i], mirrored, delta);
		}
		return velocities;
	}

	public Vector3 GetBoneVelocity(string bone, bool mirrored, float delta) {
		return GetBoneVelocity(Data.Source.FindBone(bone).Index, mirrored, delta);
	}

	public Vector3 GetBoneVelocity(int index, bool mirrored, float delta) {
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp - delta < 0f) {
			return (Data.GetFrame(Timestamp + delta).GetBoneTransformation(index, mirrored).GetPosition() - GetBoneTransformation(index, mirrored).GetPosition()) / delta;
		} else {
			return (GetBoneTransformation(index, mirrored).GetPosition() - Data.GetFrame(Timestamp - delta).GetBoneTransformation(index, mirrored).GetPosition()) / delta;
		}
	}

	public Vector3[] GetBoneAccelerations(bool mirrored, float delta) {
		Vector3[] accelerations = new Vector3[Data.Source.Bones.Length];
		for(int i=0; i<World.Length; i++) {
			accelerations[i] = GetBoneAcceleration(i, mirrored, delta);
		}
		return accelerations;
	}

	public Vector3[] GetBoneAccelerations(string[] bones, bool mirrored, float delta) {
		Vector3[] accelerations = new Vector3[bones.Length];
		for(int i=0; i<accelerations.Length; i++) {
			accelerations[i] = GetBoneAcceleration(bones[i], mirrored, delta);
		}
		return accelerations;
	}

	public Vector3 GetBoneAcceleration(string bone, bool mirrored, float delta) {
		return GetBoneAcceleration(Data.Source.FindBone(bone).Index, mirrored, delta);
	}

	public Vector3 GetBoneAcceleration(int index, bool mirrored, float delta) {
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp - delta < 0f) {
			return (Data.GetFrame(Timestamp + delta).GetBoneVelocity(index, mirrored, delta) - GetBoneVelocity(index, mirrored, delta)) / delta;
		} else {
			return (GetBoneVelocity(index, mirrored, delta) - Data.GetFrame(Timestamp - delta).GetBoneVelocity(index, mirrored, delta)) / delta;
		}
	}

	public float[] GetAngularBoneVelocities(bool mirrored, float delta) {
		float[] values = new float[Data.Source.Bones.Length];
		for(int i=0; i<World.Length; i++) {
			values[i] = GetAngularBoneVelocity(i, mirrored, delta);
		}
		return values;
	}

	public float[] GetAngularBoneVelocities(string[] bones, bool mirrored, float delta) {
		float[] values = new float[bones.Length];
		for(int i=0; i<values.Length; i++) {
			values[i] = GetAngularBoneVelocity(bones[i], mirrored, delta);
		}
		return values;
	}

	public float GetAngularBoneVelocity(string bone, bool mirrored, float delta) {
		return GetAngularBoneVelocity(Data.Source.FindBone(bone).Index, mirrored, delta);
	}

	public float GetAngularBoneVelocity(int index, bool mirrored, float delta) {
		if(delta == 0f) {
			return 0f;
		}
		if(Timestamp - delta < 0f) {
			return Quaternion.Angle(GetBoneTransformation(index, mirrored).GetRotation(), Data.GetFrame(Timestamp + delta).GetBoneTransformation(index, mirrored).GetRotation()) / delta;
		} else {
			return Quaternion.Angle(Data.GetFrame(Timestamp - delta).GetBoneTransformation(index, mirrored).GetRotation(), GetBoneTransformation(index, mirrored).GetRotation()) / delta;
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//LOCAL
	////////////////////////////////////////////////////////////////////////////////////////////////////
	public Matrix4x4[] GetLocalBoneTransformations(bool mirrored) {
		Matrix4x4[] transformations = new Matrix4x4[World.Length];
		for(int i=0; i<World.Length; i++) {
			transformations[i] = GetLocalBoneTransformation(i, mirrored);
		}
		return transformations;
	}

	public Matrix4x4[] GetLocalBoneTransformations(string[] bones, bool mirrored) {
		Matrix4x4[] transformations = new Matrix4x4[bones.Length];
		for(int i=0; i<transformations.Length; i++) {
			transformations[i] = GetLocalBoneTransformation(bones[i], mirrored);
		}
		return transformations;
	}

	public Matrix4x4 GetLocalBoneTransformation(string bone, bool mirrored) {
		return GetLocalBoneTransformation(Data.Source.FindBone(bone).Index, mirrored);
	}

	public Matrix4x4 GetLocalBoneTransformation(int index, bool mirrored) {
		if(index == 0) {
			return GetBoneTransformation(index, mirrored);
		} else {
			return GetBoneTransformation(index, mirrored).GetRelativeTransformationTo(GetBoneTransformation(Data.Source.Bones[index].Parent, mirrored));
		}
	}

	public Vector3[] GetLocalBoneVelocities(bool mirrored, float delta) {
		Vector3[] velocities = new Vector3[Data.Source.Bones.Length];
		for(int i=0; i<World.Length; i++) {
			velocities[i] = GetLocalBoneVelocity(i, mirrored, delta);
		}
		return velocities;
	}

	public Vector3[] GetLocalBoneVelocities(string[] bones, bool mirrored, float delta) {
		Vector3[] velocities = new Vector3[bones.Length];
		for(int i=0; i<velocities.Length; i++) {
			velocities[i] = GetLocalBoneVelocity(bones[i], mirrored, delta);
		}
		return velocities;
	}

	public Vector3 GetLocalBoneVelocity(string bone, bool mirrored, float delta) {
		return GetLocalBoneVelocity(Data.Source.FindBone(bone).Index, mirrored, delta);
	}

	public Vector3 GetLocalBoneVelocity(int index, bool mirrored, float delta) {
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp - delta < 0f) {
			return (Data.GetFrame(Timestamp + delta).GetLocalBoneTransformation(index, mirrored).GetPosition() - GetLocalBoneTransformation(index, mirrored).GetPosition()) / delta;
		} else {
			return (GetLocalBoneTransformation(index, mirrored).GetPosition() - Data.GetFrame(Timestamp - delta).GetLocalBoneTransformation(index, mirrored).GetPosition()) / delta;
		}
	}

	public Vector3[] GetLocalBoneAccelerations(bool mirrored, float delta) {
		Vector3[] accelerations = new Vector3[Data.Source.Bones.Length];
		for(int i=0; i<World.Length; i++) {
			accelerations[i] = GetLocalBoneAcceleration(i, mirrored, delta);
		}
		return accelerations;
	}

	public Vector3[] GetLocalBoneAccelerations(string[] bones, bool mirrored, float delta) {
		Vector3[] accelerations = new Vector3[bones.Length];
		for(int i=0; i<accelerations.Length; i++) {
			accelerations[i] = GetLocalBoneAcceleration(bones[i], mirrored, delta);
		}
		return accelerations;
	}

	public Vector3 GetLocalBoneAcceleration(string bone, bool mirrored, float delta) {
		return GetLocalBoneAcceleration(Data.Source.FindBone(bone).Index, mirrored, delta);
	}

	public Vector3 GetLocalBoneAcceleration(int index, bool mirrored, float delta) {
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp - delta < 0f) {
			return (Data.GetFrame(Timestamp + delta).GetLocalBoneVelocity(index, mirrored, delta) - GetLocalBoneVelocity(index, mirrored, delta)) / delta;
		} else {
			return (GetLocalBoneVelocity(index, mirrored, delta) - Data.GetFrame(Timestamp - delta).GetLocalBoneVelocity(index, mirrored, delta)) / delta;
		}
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////
	//HAND
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// hand transformation
	public Matrix4x4 GetHandWristTransformation(int handIndex, bool mirrored){
		if(Data.Source.FindBone(wristName)!=null){
			return GetHandTransformation(handIndex, Data.Source.FindBone(wristName).Index, mirrored);
		}
		else{
			Debug.LogError("can not find WRIST");
			return Matrix4x4.identity;
		}
	}
	public Matrix4x4 GetHandTransformation(int handIndex, string bone, bool mirrored) {
		return GetHandTransformation(handIndex, Data.Source.FindBone(bone).Index, mirrored);
	}
	public Matrix4x4 GetHandTransformation(int handIndex, int boneIndex, bool mirrored) {
		int numJoint = Data.Source.Bones.Length;
		Matrix4x4 m = mirrored ? World[numJoint*handIndex + boneIndex].GetMirror(Data.MirrorAxis) : World[numJoint*handIndex + boneIndex];
		// Matrix4x4 m = mirrored ? World[index].GetMirror(Data.MirrorAxis) : World[index];
		Vector3 o = mirrored ? Data.Offset.GetMirror(Data.MirrorAxis) : Data.Offset;
		m[0,3] += o.x;
		m[1,3] += o.y;
		m[2,3] += o.z;
		return m;
	}
	public Matrix4x4[] GetHandTransformations(int handIndex, bool mirrored){
		int numJoint = Data.Source.Bones.Length;
		Matrix4x4[] handTransformations = new Matrix4x4[numJoint];
		for(int iJoint=0; iJoint<numJoint; iJoint++){
			handTransformations[iJoint] = GetHandTransformation(handIndex, iJoint, mirrored);
		}
		return handTransformations;
	}
	
	// hand joint local transformation
	public Matrix4x4[] GetHandLocalTransformations(int handIndex, bool mirrored){
		int numJoint = Data.Source.Bones.Length;
		Matrix4x4[] handLocalTranses = new Matrix4x4[numJoint];
		for(int iJoint=0; iJoint<numJoint; iJoint++){
			if(iJoint == 0) {
				handLocalTranses[iJoint] = GetHandTransformation(handIndex, iJoint, mirrored);
			}
			else{
				handLocalTranses[iJoint] = GetHandTransformation(handIndex, iJoint, mirrored).GetRelativeTransformationTo(
											GetHandTransformation(handIndex, Data.Source.Bones[iJoint].Parent, mirrored));
			}
		}
		return handLocalTranses;
	}

	// for testing - use joint angles to get the joint local rotations
	public Quaternion[] GetHandLocalRotations_JointAngles(int handIndex){
		Matrix4x4[] restPose = Data.restLocalPose; // rest pose
		int nJoints = restPose.Length/2;  
		Quaternion[] handLocalRotations = new Quaternion[nJoints];
		int[] dofs = Data.Dofs; // dofs for each joint
		int nDofs = dofs.Sum();
		if(nJoints!=dofs.Length){
			Debug.LogError("joint number " + nJoints + " and dof length " + dofs.Length + " are not matching!");
			return handLocalRotations;
		}
		if(nDofs!=jointAngles.Length/2){
			Debug.LogError("dof number " + nDofs+ " and joint angle length " + jointAngles.Length/2 + "are not matching!");
			return handLocalRotations;
		}
		int dofIndex = 0;
		for(int iJoint=0; iJoint<nJoints; iJoint++){
			handLocalRotations[iJoint] = GetJointLocalRotation_JointAngles(handIndex, dofIndex, dofs[iJoint], restPose[iJoint+nJoints*handIndex]);
			dofIndex += dofs[iJoint];
		}
		return handLocalRotations;
	}
	public Quaternion GetJointLocalRotation_JointAngles(int handIndex, int dofIndex, int dofs,  Matrix4x4 jointRestT){
		Quaternion localRotation = Quaternion.identity;
		// int m = mirrored? 1:0;
		// dofIndex = dofIndex + (int)Math.Abs(handIndex-m)*jointAngles.Length/2;  // if non-mirror and right hand add OR if mirror left hand add
		// m = mirrored? -1:1;  // if mirrored *-1 for the joint angles
		
		int m = handIndex==0? -1:1;
		dofIndex = dofIndex + (int)handIndex*jointAngles.Length/2; 
		if(dofs==1){
			float rotationZ = m * (float)jointAngles[dofIndex] * Mathf.Rad2Deg;
        	localRotation = jointRestT.GetRotation() * Quaternion.Euler(0f, 0f, rotationZ);
		}
		else if(dofs==2){
			float rotationZ =  m * (float)jointAngles[dofIndex] * Mathf.Rad2Deg ;
			float rotationY =  m * (float)jointAngles[dofIndex+1] * Mathf.Rad2Deg;
			localRotation = jointRestT.GetRotation() * Quaternion.Euler(0f, rotationY, rotationZ);
		}
		else{
			Debug.LogError("wrong dof, neither 1 or 2");
		}
		return localRotation;
	}

	// hand velocity 
	public Vector3 GetWristVelocity(int handIndex, bool mirrored, float delta){
		if(Data.Source.FindBone(wristName)!=null){
			return GetHandVelocity(handIndex, Data.Source.FindBone(wristName).Index, mirrored, delta);
		}
		else{
			Debug.LogError("can not find WRIST");
			return Vector3.zero;
		}
	}
	public Vector3 GetHandVelocity(int handIndex, string bone, bool mirrored, float delta) {
		return GetHandVelocity(handIndex, Data.Source.FindBone(bone).Index, mirrored, delta);
	}
	public Vector3 GetHandVelocity(int handIndex, int boneIndex, bool mirrored, float delta) {
		// here the velocity is calculated in the wrist space, we finally transform this in to the world space
		if(delta == 0f) {
			return Vector3.zero;
		}
		Vector3 vel = Vector3.zero; 
		Matrix4x4 transA = Matrix4x4.identity;
		Matrix4x4 transB = Matrix4x4.identity;
		float totalTime = Data.GetTotalTime();
		if(Timestamp + delta > totalTime) {
			if(boneIndex==0){
				transA = Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, boneIndex, mirrored);
				transB = GetHandTransformation(handIndex, boneIndex, mirrored);
			}
			else{
				transA = Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, boneIndex, mirrored).GetRelativeTransformationTo(Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, 0, mirrored));
				transB = GetHandTransformation(handIndex, boneIndex, mirrored).GetRelativeTransformationTo(GetHandTransformation(handIndex, 0, mirrored));
			}
			
		} else {
			if(boneIndex==0){
				transA = GetHandTransformation(handIndex, boneIndex, mirrored);
				transB = Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, boneIndex, mirrored);
			}
			else{
				transA = GetHandTransformation(handIndex, boneIndex, mirrored).GetRelativeTransformationTo(GetHandTransformation(handIndex, 0, mirrored));
				transB = Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, boneIndex, mirrored).GetRelativeTransformationTo(Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, 0, mirrored));
			}
		}
		// finally transform this into world space
		vel = (transB.GetPosition()-transA.GetPosition())/delta;
		if(boneIndex==0){
			return vel;
		}
		return vel.GetRelativeDirectionFrom(GetHandTransformation(handIndex, 0, mirrored));
	}
	public Vector3[] GetHandVelocities(int handIndex, bool mirrored, float delta){
		int numJoint = Data.Source.Bones.Length;
		Vector3[] velocities = new Vector3[numJoint];
		for(int iJoint=0; iJoint<numJoint; iJoint++){
			velocities[iJoint] = GetHandVelocity(handIndex, iJoint, mirrored, delta);
		}
		return velocities;
	}

	// hand angular velocity
	public float[] GetWristAngularVelocity(int handIndex, bool mirrored, float delta){
		if(Data.Source.FindBone(wristName)!=null){
			return GetHandAngularVelocity(handIndex, Data.Source.FindBone(wristName).Index, mirrored, delta);
		}
		else{
			Debug.LogError("can not find WRIST");
			return new float[dimAngularVelocity];
		}
	}
	public float[] GetHandAngularVelocity(int handIndex, int boneIndex, bool mirrored, float delta) {
		// here return the NewForwardvector - OldForwardVector and Up
		float[] linearAngularVelocity = new float[dimAngularVelocity];
		Vector3 forwardAngularSpeed = Vector3.one;
		Vector3 upAngularSpeed = Vector3.one;
		float totalTime = Data.GetTotalTime();
		if(Timestamp + delta > totalTime) {	
			forwardAngularSpeed = GetHandTransformation(handIndex, boneIndex, mirrored).GetForward() 
								  - Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, boneIndex, mirrored).GetForward();
			upAngularSpeed = GetHandTransformation(handIndex, boneIndex, mirrored).GetUp() 
							 - Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, boneIndex, mirrored).GetUp();
		} else {
			forwardAngularSpeed = Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, boneIndex, mirrored).GetForward() 
								  - GetHandTransformation(handIndex, boneIndex, mirrored).GetForward();
			upAngularSpeed = Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, boneIndex, mirrored).GetUp() 
								  - GetHandTransformation(handIndex, boneIndex, mirrored).GetUp();
			
		}

		for(int i=0; i<(int)dimAngularVelocity/2; i++){
			linearAngularVelocity[i] = forwardAngularSpeed[i];
		}
		for(int i=0; i<(int)dimAngularVelocity/2; i++){
			linearAngularVelocity[i+(int)dimAngularVelocity/2] = upAngularSpeed[i];
		}
		return linearAngularVelocity;
	}

	// public float GetHandAngularVelocity(int handIndex, int boneIndex, bool mirrored, float delta, Axis axis) {
	// 	// return (-pi, pi)
	// 	float angularV = 0f;
	// 	Matrix4x4 transA = Matrix4x4.identity;
	// 	Matrix4x4 transB = Matrix4x4.identity;
	// 	Vector3 vectorA = Vector3.zero;
	// 	Vector3 vectorB = Vector3.zero;
	// 	if(Timestamp - delta < 0f) {
	// 		transA = GetHandTransformation(handIndex, boneIndex, mirrored);
	// 		transB = Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, boneIndex, mirrored);
	// 	}
	// 	else{
	// 		transA = Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, boneIndex, mirrored);
	// 		transB = GetHandTransformation(handIndex, boneIndex, mirrored);
	// 	}
	// 	switch(axis){
	// 			case Axis.XPositive:
	// 			// right 
	// 			vectorA = transA.GetRight();
	// 			vectorB = transB.GetRight();
	// 			break;
	// 			case Axis.YPositive:
	// 			// up
	// 			vectorA = transA.GetUp();
	// 			vectorB = transB.GetUp();
	// 			break;
	// 			case Axis.ZPositive:
	// 			// forward
	// 			vectorA = transA.GetForward();
	// 			vectorB = transB.GetForward();
	// 			break;
	// 		}
	// 	angularV = Vector3.SignedAngle(vectorA, vectorB, Vector3.Cross(vectorA, vectorB));
	// 	return angularV;
	// }
	
	// Tips 
	public Vector3 GetHandTipPosition(int handIndex, int tipIndex, bool mirror){
		int nTips = 5;
		if(tipWorldPositions==null){
			Debug.LogError("tip positions are not baked");
			return Vector3.zero;
		}
		if(tipWorldPositions.Length!=nTips*2){
			Debug.LogError("total tip positions number is wrong " + tipWorldPositions.Length);
			return Vector3.zero;
		}
		Axis mirrorAxis = mirror? Data.MirrorAxis:Axis.None;
		return tipWorldPositions[handIndex*nTips+tipIndex].GetMirror(mirrorAxis);
	}
	public Vector3[] GetHandTipPositions(int handIndex, bool mirror){
		int nTips = 5;
		Vector3[] tipPositions = new Vector3[nTips];
		for(int iTip=0; iTip<nTips; iTip++){
			tipPositions[iTip] = GetHandTipPosition(handIndex, iTip, mirror);
		}
		return tipPositions;
	}
	public Matrix4x4[] GetTipTransformations(int handIndex, bool mirror){
		int nTips = 5;
		Matrix4x4[] tipTransformations = new Matrix4x4[nTips];
		for(int iTip=0; iTip<nTips; iTip++){
			tipTransformations[iTip] = Matrix4x4.TRS(GetHandTipPosition(handIndex, iTip, mirror), Quaternion.identity, Vector3.one);
		}
		return tipTransformations;
	}
	public Vector3[] GetHandVisualTipPositions(int handIndex, bool mirror){
		handIndex = mirror? Math.Abs(handIndex-1):handIndex;
		return GetHandTipPositions(handIndex, mirror);
	}

	public Vector3 GetHandTipVelocity(int handIndex, int tipIndex, bool mirrored, float delta) {
		// here the velocity is calculated in the wrist space, we finally transform this into the world space
		if(delta == 0f) {
			return Vector3.zero;
		}
		Vector3 vel = Vector3.zero; 
		Vector3 posA = Vector3.zero;
		Vector3 posB = Vector3.zero;
		float totalTime = Data.GetTotalTime();
		if(Timestamp + delta > totalTime) {	
			posA = Data.GetFrame(Timestamp - delta).GetHandTipPosition(handIndex, tipIndex, mirrored).GetRelativePositionTo(Data.GetFrame(Timestamp - delta).GetHandTransformation(handIndex, 0, mirrored));
			posB = GetHandTipPosition(handIndex, tipIndex, mirrored).GetRelativePositionTo(GetHandTransformation(handIndex, 0, mirrored));
		} else {
			posA = GetHandTipPosition(handIndex, tipIndex, mirrored).GetRelativePositionTo(GetHandTransformation(handIndex, 0, mirrored));
			posB = Data.GetFrame(Timestamp + delta).GetHandTipPosition(handIndex, tipIndex, mirrored).GetRelativePositionTo(Data.GetFrame(Timestamp + delta).GetHandTransformation(handIndex, 0, mirrored));
		}
		// finally transform this into world space\
		vel = (posB-posA)/delta;
		return vel.GetRelativeDirectionFrom(GetHandTransformation(handIndex, 0, mirrored));
	}
	public Vector3[] GetHandTipVelocities(int handIndex, bool mirrored, float delta){
		int nTips = 5;
		Vector3[] velocities = new Vector3[nTips];
		for(int iTip=0; iTip<nTips; iTip++){
			velocities[iTip] = GetHandTipVelocity(handIndex, iTip, mirrored, delta);
		}
		return velocities;
	}
	public Vector3[] GetHandVisualTipVelocities(int handIndex, bool mirrored, float delta){
		handIndex = mirrored? Math.Abs(handIndex-1):handIndex;
		return GetHandTipVelocities(handIndex, mirrored, delta);
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////
	//OBJECT
	////////////////////////////////////////////////////////////////////////////////////////////////////
	public Matrix4x4[] GetObjectTransformations(bool mirrored) {
		Matrix4x4[] objTrans = new Matrix4x4[numObjects];
		for(int iObj=0; iObj<numObjects; iObj++){
			objTrans[iObj] = mirrored ? objectsWorld[iObj].GetMirror(Data.MirrorAxis):objectsWorld[iObj];
			Vector3 o = mirrored ? Data.Offset.GetMirror(Data.MirrorAxis) : Data.Offset;
			objTrans[iObj][0,3] += o.x;
			objTrans[iObj][1,3] += o.y;
			objTrans[iObj][2,3] += o.z;
		}	
		return objTrans;
	}

	public Matrix4x4 GetObjectTransformation(int objectIndex, bool mirrored) {
		Matrix4x4 m = mirrored ? objectsWorld[objectIndex].GetMirror(Data.MirrorAxis):objectsWorld[objectIndex];
		Vector3 o = mirrored ? Data.Offset.GetMirror(Data.MirrorAxis) : Data.Offset;
		m[0,3] += o.x;
		m[1,3] += o.y;
		m[2,3] += o.z;
		return m;
	}
	public Vector3[] GetObjectPositions(bool mirrored){
		Vector3[] objectsP = new Vector3[numObjects];
		for(int iObj=0; iObj<numObjects; iObj++){
			objectsP[iObj] = GetObjectTransformation(iObj, mirrored).GetPosition();
		}
		return objectsP;
	}

	public Vector3 GetObjectWorldVelocity(int objectIndex, bool mirrored, float delta) {
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp - delta < 0f) {
			return (Data.GetFrame(Timestamp + delta).GetObjectTransformation(objectIndex, mirrored).GetPosition() - GetObjectTransformation(objectIndex, mirrored).GetPosition()) / delta;
		} else {
			return (GetObjectTransformation(objectIndex, mirrored).GetPosition() - Data.GetFrame(Timestamp - delta).GetObjectTransformation(objectIndex, mirrored).GetPosition()) / delta;
		}
	}
	public Matrix4x4 GetObjectRelativeTransV(int objectIndex, int handIndex, bool mirrored, float delta){
		// first get the object trans relative to wrist trans at 2 adjacent frames, then get the difference for getting the rotation
		Matrix4x4 transA = Matrix4x4.identity;
		Matrix4x4 transB = Matrix4x4.identity;
		Matrix4x4 transHandA = Matrix4x4.identity;
		Matrix4x4 transHandB = Matrix4x4.identity;
		if(Timestamp - delta < 0f) {
			transA = GetObjectTransformation(objectIndex, mirrored);
			transB = Data.GetFrame(Timestamp + delta).GetObjectTransformation(objectIndex, mirrored);
			transHandA = GetHandWristTransformation(handIndex, mirrored);
			transHandB = Data.GetFrame(Timestamp + delta).GetHandWristTransformation(handIndex, mirrored);
		}
		else{
			transA = Data.GetFrame(Timestamp - delta).GetObjectTransformation(objectIndex, mirrored);
			transB = GetObjectTransformation(objectIndex, mirrored);
			transHandA = Data.GetFrame(Timestamp - delta).GetHandWristTransformation(handIndex, mirrored);
			transHandB = GetHandWristTransformation(handIndex, mirrored);
		}
		transA = transA.GetRelativeTransformationTo(transHandA);
		transB = transB.GetRelativeTransformationTo(transHandB);

		/// <summary>
		/// transA t-1
		/// transB t
		/// trans = transA.inverse*transB;
		/// trans.forward
		/// trans.up
		/// </summary>

		Matrix4x4 trans = transA.inverse*transB;
		return trans;
	}


	public Vector3 GetObjectRelativeAxisAngleV(int objectIndex, int handIndex, bool mirrored, float delta) {
		Vector3 angularV = Vector3.zero;
		float angle = 0f;
		Matrix4x4 trans = GetObjectRelativeTransV(objectIndex, handIndex, mirrored, delta);
		Quaternion rot = trans.GetRotation();
		rot.ToAngleAxis(out angle, out angularV);
		angularV = angle/180f*Mathf.PI/delta*angularV;
		return angularV;
	}

	public static Vector3 StaticGetObjectRelativeAxisAngleV(Matrix4x4[] objectTranses, Matrix4x4[] handTranses, float delta) {
		/// <summary>
		/// here is static methods for getting the axis angle in wrist space, similar as GetObjectRelativeAxisAngleV	
		/// given the world transformation of the wrist and object at two frames A, B 
		/// A, B are two frames with delta interval
		/// </summary>
		/// <returns></returns>
		Matrix4x4 transA = objectTranses[0].GetRelativeTransformationTo(handTranses[0]);
		Matrix4x4 transB = objectTranses[1].GetRelativeTransformationTo(handTranses[1]);
		Matrix4x4 trans = transA.inverse*transB;
		
		Vector3 angularV = Vector3.zero;
		float angle = 0f;
		Quaternion rot = trans.GetRotation();
		rot.ToAngleAxis(out angle, out angularV);
		angularV = angle/180f*Mathf.PI/delta*angularV;
		return angularV;
	}


	// ========================= object linear and angular velocities and acceleration =========================
	/*
	a little bit different from above calculation where it is t-(t-1), because run time can only get current and previous frames
	here I wish to do a bit different for pre-computation - using (t+1)-t and make sure everything is in the object local coordinate at current frame 
	*/

	public Vector3 GetObjectWorldLinearV(int objectIndex, bool mirrored, float delta) {
		// linear velocilty
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp + delta > Data.GetTotalTime()) {
			return (GetObjectTransformation(objectIndex, mirrored).GetPosition() - Data.GetFrame(Timestamp-delta).GetObjectTransformation(objectIndex, mirrored).GetPosition()) / delta;
		} else {
			return (Data.GetFrame(Timestamp+delta).GetObjectTransformation(objectIndex, mirrored).GetPosition() - GetObjectTransformation(objectIndex, mirrored).GetPosition()) / delta;
		}
	}
	public Vector3 GetObjectWorldLinearA(int objectIndex, bool mirrored, float delta) {
		// linear acceleration
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp + delta > Data.GetTotalTime()) {
			return (GetObjectWorldLinearV(objectIndex, mirrored, delta) - Data.GetFrame(Timestamp-delta).GetObjectWorldLinearV(objectIndex, mirrored, delta));
		} else {
			return (Data.GetFrame(Timestamp+delta).GetObjectWorldLinearV(objectIndex, mirrored, delta) - GetObjectWorldLinearV(objectIndex, mirrored, delta));
		}
	}
	public Vector3 GetObjectWolrdAngularV(int objectIndex, bool mirrored, float delta) { 
		// axis angle
		if(delta == 0f) {
			return Vector3.zero;
		}
		Matrix4x4 transA = Matrix4x4.identity;
		Matrix4x4 transB = Matrix4x4.identity;
		if(Timestamp + delta > Data.GetTotalTime()) {
			transA = Data.GetFrame(Timestamp - delta).GetObjectTransformation(objectIndex, mirrored);
			transB = GetObjectTransformation(objectIndex, mirrored);
		}
		else{
			transA = GetObjectTransformation(objectIndex, mirrored);
			transB = Data.GetFrame(Timestamp + delta).GetObjectTransformation(objectIndex, mirrored);
		}
		Matrix4x4 trans = transA.inverse*transB;
		Vector3 angularV = Vector3.zero;
		float angle = 0f;
		Quaternion rot = trans.GetRotation();
		rot.ToAngleAxis(out angle, out angularV);
		angularV = angle/180f*Mathf.PI/delta*angularV;
		return angularV;
	}
	public Vector3 GetObjectWolrdAngularA(int objectIndex, bool mirrored, float delta) {
		if(delta == 0f) {
			return Vector3.zero;
		}
		if(Timestamp + delta > Data.GetTotalTime()) {
			return (GetObjectWolrdAngularV(objectIndex, mirrored, delta)-Data.GetFrame(Timestamp-delta).GetObjectWolrdAngularV(objectIndex, mirrored, delta));
		} 
		else {
			return (Data.GetFrame(Timestamp+delta).GetObjectWolrdAngularV(objectIndex, mirrored, delta)-GetObjectWolrdAngularV(objectIndex, mirrored, delta));
		}
	}
	// ========================= object linear and angular velocities and acceleration =========================
}