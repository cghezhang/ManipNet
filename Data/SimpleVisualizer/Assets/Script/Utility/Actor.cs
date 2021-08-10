using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteInEditMode]
public class Actor : MonoBehaviour {
	public const string MeshBoneName = "MeshBone";
	public int Index = 0;

	public bool InspectSkeleton = false;
	
	public bool DrawRoot = false;
	public bool DrawSkeleton = false;
	public bool DrawTransforms = false;
	public bool DrawVelocities = false;
	public bool DrawAccelerations = false;
	public bool DrawForces = false;
	public bool DrawHistory = false;

	public int MaxHistory = 0;

	public float BoneSize = 0.025f;
	public Color BoneColor = UltiDraw.Black;
	public Color JointColor = UltiDraw.Mustard;
	public Color colliderColor = UltiDraw.Green;

	public Bone[] Bones = new Bone[0];

	private List<State> History = new List<State>();

	void Reset() {
		ExtractSkeleton();
	}

	void Awake() {
		if(Application.isPlaying) {
			for(int i=0; i<Bones.Length; i++) {
				Bones[i].ComputeLength();
			}
		}
	}

	void LateUpdate() {
		SaveState();
	}

	public void SaveState() {
		if(MaxHistory > 0) {
			State state = new State();
			state.Transformations = GetBoneTransformations();
			state.Velocities = GetBoneVelocities();
			state.Accelerations = GetBoneAccelerations();
			History.Add(state);
		}
		while(History.Count > MaxHistory) {
			History.RemoveAt(0);
		}
	}

	public Transform GetRoot() {
		return transform;
	}

	public Transform[] FindTransforms(params string[] names) {
		Transform[] transforms = new Transform[names.Length];
		for(int i=0; i<transforms.Length; i++) {
			transforms[i] = FindTransform(names[i]);
		}
		return transforms;
	}

	public Transform FindTransform(string name) {
		Transform element = null;
		Action<Transform> recursion = null;
		recursion = new Action<Transform>((transform) => {
			if(transform.name == name) {
				element = transform;
				return;
			}
			for(int i=0; i<transform.childCount; i++) {
				recursion(transform.GetChild(i));
			}
		});
		recursion(GetRoot());
		return element;
	}

	public Bone[] FindBones(params Transform[] transforms) {
		Bone[] bones = new Bone[transforms.Length];
		for(int i=0; i<bones.Length; i++) {
			bones[i] = FindBone(transforms[i]);
		}
		return bones;
	}

	public Bone[] FindBones(params string[] names) {
		Bone[] bones = new Bone[names.Length];
		for(int i=0; i<bones.Length; i++) {
			bones[i] = FindBone(names[i]);
		}
		return bones;
	}

	public Bone FindBone(Transform transform) {
		return FindBone(transform.name);
	}

	public Bone FindBone(string name) {
		return Array.Find(Bones, x => x.GetName() == name);
	}

	public Bone FindBoneContains(string name) {
		return Array.Find(Bones, x => x.GetName().Contains(name));
	}

	public string[] GetBoneNames() {
		string[] names = new string[Bones.Length];
		for(int i=0; i<names.Length; i++) {
			names[i] = Bones[i].GetName();
		}
		return names;
	}

	public int[] GetBoneIndices(params string[] names) {
		int[] indices = new int[names.Length];
		for(int i=0; i<indices.Length; i++) {
			indices[i] = FindBone(names[i]).Index;
		}
		return indices;
	}

	public Transform[] GetBoneTransforms(params string[] names) {
		Transform[] transforms = new Transform[names.Length];
		for(int i=0; i<names.Length; i++) {
			transforms[i] = FindTransform(names[i]);
		}
		return transforms;
	}

	public Transform[] GetBoneTransforms(){
		List<Transform> boneTransforms = new List<Transform>();
		// original bones
		for(int iBone=0; iBone<this.Bones.Length; iBone++){
			boneTransforms.Add(this.Bones[iBone].Transform);
		}
		return boneTransforms.ToArray();
	}

	public void RemoveBoneFromSkeleton(int boneIndex){
		Transform[] bones = new Transform[Bones.Length];
		for(int i=0; i<bones.Length; i++) {
			bones[i] = Bones[i].Transform;
		}
		if(boneIndex>=0 && boneIndex<bones.Length){
			ArrayExtensions.RemoveAt(ref bones, boneIndex);
			ExtractSkeleton(bones);
		}
	}

	public void ExtractSkeleton() {
		ArrayExtensions.Clear(ref Bones);
		Action<Transform, Bone> recursion = null;
		recursion = new Action<Transform, Bone>((transform, parent) => {
			Bone bone = new Bone(this, transform, Bones.Length);
			ArrayExtensions.Add(ref Bones, bone);
			if(parent != null) {
				bone.Parent = parent.Index;
				ArrayExtensions.Add(ref parent.Childs, bone.Index);
			}
			parent = bone;
			for(int i=0; i<transform.childCount; i++) {
				recursion(transform.GetChild(i), parent);
			}
		});
		recursion(GetRoot(), null);
	}

	public void ExtractSkeleton(Transform[] bones) {
		ArrayExtensions.Clear(ref Bones);
		Action<Transform, Bone> recursion = null;
		recursion = new Action<Transform, Bone>((transform, parent) => {
			if(System.Array.Find(bones, x => x == transform)) {
				Bone bone = new Bone(this, transform, Bones.Length);
				ArrayExtensions.Add(ref Bones, bone);
				if(parent != null) {
					bone.Parent = parent.Index;
					ArrayExtensions.Add(ref parent.Childs, bone.Index);
				}
				parent = bone;
			}
			for(int i=0; i<transform.childCount; i++) {
				recursion(transform.GetChild(i), parent);
			}
		});
		recursion(GetRoot(), null);
	}

	public void ExtractSkeleton(string[] bones) {
		ExtractSkeleton(FindTransforms(bones));
	}

	public void WriteTransforms(Matrix4x4[] values, string[] names) {
		Action<Transform> recursion = null;
		recursion = new Action<Transform>((t) => {
			int index = ArrayExtensions.FindIndex(ref names, t.name);
			if(index > -1) {
				t.position = values[index].GetPosition();
				t.rotation = values[index].GetRotation();
			}
			for(int i=0; i<t.childCount; i++) {
				recursion(t.GetChild(i));
			}
		});
		recursion(transform);
	}

	public void SetBoneTransformations(Matrix4x4[] values) {
		if(values.Length != Bones.Length) {
			return;
		}
		for(int i=0; i<Bones.Length; i++) {
			Bones[i].Transform.position = values[i].GetPosition();
			Bones[i].Transform.rotation = values[i].GetRotation();
		}
	}

	public void SetBoneTransformations(Matrix4x4[] values, params string[] bones) {
		for(int i=0; i<bones.Length; i++) {
			SetBoneTransformation(values[i], bones[i]);
		}
	}

	public void SetBoneTransformation(Matrix4x4 value, string bone) {
		Bone b = FindBone(bone);
		if(b != null) {
			b.Transform.position = value.GetPosition();
			b.Transform.rotation = value.GetRotation();
		}
	}

	public Matrix4x4[] GetBoneTransformations() {
		Matrix4x4[] transformations = new Matrix4x4[Bones.Length];
		for(int i=0; i<transformations.Length; i++) {
			transformations[i] = Bones[i].Transform.GetWorldMatrix();
		}
		return transformations;
	}

	public Matrix4x4[] GetBoneTransformations(params string[] bones) {
		Matrix4x4[] transformations = new Matrix4x4[bones.Length];
		for(int i=0; i<transformations.Length; i++) {
			transformations[i] = GetBoneTransformation(bones[i]);
		}
		return transformations;
	}

	public Matrix4x4 GetBoneTransformation(string bone) {
		return FindBone(bone).Transform.GetWorldMatrix();
	}

	public void SetBoneVelocities(Vector3[] values) {
		if(values.Length != Bones.Length) {
			return;
		}
		for(int i=0; i<Bones.Length; i++) {
			Bones[i].Velocity = values[i];
		}
	}

	public void SetBoneVelocities(Vector3[] values, params string[] bones) {
		for(int i=0; i<bones.Length; i++) {
			SetBoneVelocity(values[i], bones[i]);
		}
	}

	public void SetBoneVelocity(Vector3 value, string bone) {
		Bone b = FindBone(bone);
		if(b != null) {
			b.Velocity = value;
		}
	}

	public Vector3[] GetBoneVelocities() {
		Vector3[] velocities = new Vector3[Bones.Length];
		for(int i=0; i<velocities.Length; i++) {
			velocities[i] = Bones[i].Velocity;
		}
		return velocities;
	}

	public Vector3[] GetBoneVelocities(params string[] bones) {
		Vector3[] velocities = new Vector3[bones.Length];
		for(int i=0; i<velocities.Length; i++) {
			velocities[i] = GetBoneVelocity(bones[i]);
		}
		return velocities;
	}

	public Vector3 GetBoneVelocity(string bone) {
		return FindBone(bone).Velocity;
	}

	public Vector3[] GetBoneAccelerations() {
		Vector3[] values = new Vector3[Bones.Length];
		for(int i=0; i<Bones.Length; i++) {
			values[i] = Bones[i].Acceleration;
		}
		return values;
	}

	public Vector3[] GetBoneAccelerations(params string[] bones) {
		Vector3[] values = new Vector3[bones.Length];
		for(int i=0; i<bones.Length; i++) {
			values[i] = GetBoneAcceleration(bones[i]);
		}
		return values;
	}

	public Vector3 GetBoneAcceleration(string bone) {
		return FindBone(bone).Acceleration;
	}

	//=====dof========
	public void SetDofs(int[] dofs){
		if(dofs.Length!=Bones.Length){
			Debug.LogError("array lengths between dof and bone are not matching");
			return;
		}
		for(int iBone=0; iBone<Bones.Length; iBone++){
			Bones[iBone].Dof = dofs[iBone];
		}
	}
	public int[] GetDofs(){
		int[] dofs = new int[Bones.Length];
		for(int iBone=0; iBone<Bones.Length; iBone++){
			dofs[iBone] = Bones[iBone].Dof;
		}
		return dofs;
	}

	//===================================this is for adding mesh bone for IK===========================================
	public void AddMeshBones(int[] indexParentJoints, Vector3[] meshBonePositions = null){
		if(meshBonePositions!=null && meshBonePositions.Length!=indexParentJoints.Length){
			Debug.LogError("wrong number of initial position when add mesh bones");
			return;
		}
		int iParent = 0;
		for(int i=0; i<indexParentJoints.Length; i++){
			iParent = indexParentJoints[i];
			if(iParent<this.Bones.Length){
				Transform parentBone = this.Bones[iParent].Transform;
				string meshBoneName = MeshBoneName+iParent;
				if(parentBone.Find(meshBoneName) == null){
					GameObject newMeshBone = new GameObject(meshBoneName);
					newMeshBone.transform.parent = parentBone;
					if(meshBonePositions!=null){
						newMeshBone.transform.position = meshBonePositions[i];
					}
				}				
			}
			else{
				Debug.LogError("index of the parent joint is out of boundary");
			}
		}
			

		// we only need a virtual mesh bone, so if it is activated in the real skeleton, need to remove 
		Transform[] bones = new Transform[this.Bones.Length];
		for(int iBone=0; iBone<bones.Length; iBone++) {
			bones[iBone] = this.Bones[iBone].Transform;
		}
		for(int iBone=0; iBone<this.Bones.Length; iBone++) {
			if(this.Bones[iBone].GetName().Contains(MeshBoneName)){
				ArrayExtensions.Remove(ref bones, this.Bones[iBone].Transform);
			}
		}
		this.ExtractSkeleton(bones);
	}

	public void DeleteAllMeshBoneTransform(){
		foreach (Transform child in this.Bones[0].Transform.GetComponentsInChildren<Transform>())
        {
            if(child.name.Contains(MeshBoneName)){
				GameObject.Destroy(child.gameObject);
			}
        }
	}

	public Transform[] GetMeshBoneTransforms(int[] indexParentJoints){
		List<Transform> meshBoneTransforms = new List<Transform>();
		foreach(int iParent in indexParentJoints){
			if(iParent<this.Bones.Length){
				Transform parentBone = this.Bones[iParent].Transform;
				string meshBoneName = MeshBoneName+iParent;
				if(parentBone.Find(meshBoneName) == null){
					Debug.LogError("Joint " + iParent + " has no mesh bone");
				}			
				else{
					meshBoneTransforms.Add(parentBone.Find(meshBoneName));
				}	
			}
			else{
				Debug.LogError("index of the parent joint is out of boundary");
			}
		}
		return meshBoneTransforms.ToArray();
	}

	public Transform[] GetOriginAndMeshBoneTransforms(int[] meshBoneParentIndex){
		List<Transform> boneTransforms = new List<Transform>();
		// original bones
		for(int iBone=0; iBone<this.Bones.Length; iBone++){
			boneTransforms.Add(this.Bones[iBone].Transform);
		}
		foreach(int iParent in meshBoneParentIndex){
			if(iParent<this.Bones.Length){
				Transform parentBone = this.Bones[iParent].Transform;
				string meshBoneName = MeshBoneName+iParent;
				if(parentBone.Find(meshBoneName) == null){
					Debug.LogError("Joint " + iParent + " has no mesh bone");
				}			
				else{
					boneTransforms.Add(parentBone.Find(meshBoneName));
				}	
			}
			else{
				Debug.LogError("index of the parent joint is out of boundary");
			}
		}
		return boneTransforms.ToArray();
	}


	// public Transform[] GetMeshBoneTransforms(){
	// 	List<Transform> meshBoneTransforms = new List<Transform>();
	// 	foreach (Transform child in this.Bones[0].Transform.GetComponentsInChildren<Transform>())
    //     {
    //         if(child.name.Contains(MeshBoneName)){
	// 			meshBoneTransforms.Add(child);
	// 		}
    //     }
	// 	return meshBoneTransforms.ToArray();
	// }

	public Vector3[] GetOriginAndMeshBonePositions(int[] meshBoneParentIndex){
		List<Vector3> boneTransforms = new List<Vector3>();
		// original bones
		for(int iBone=0; iBone<this.Bones.Length; iBone++){
			boneTransforms.Add(this.Bones[iBone].Transform.position);
		}
		foreach(int iParent in meshBoneParentIndex){
			if(iParent<this.Bones.Length){
				Transform parentBone = this.Bones[iParent].Transform;
				string meshBoneName = MeshBoneName+iParent;
				if(parentBone.Find(meshBoneName) == null){
					Debug.LogError("Joint " + iParent + " has no mesh bone");
				}			
				else{
					boneTransforms.Add(parentBone.Find(meshBoneName).position);
				}	
			}
			else{
				Debug.LogError("index of the parent joint is out of boundary");
			}
		}
		return boneTransforms.ToArray();
	}

	//===================================this is for adding bone mesh===========================================

	public void Draw() {
		Draw(BoneColor, JointColor, 1f);
	}

	public void Draw(Color boneColor, Color jointColor, float alpha) {
		UltiDraw.Begin();
		if(DrawRoot) {
			UltiDraw.DrawWiredSphere(GetRoot().position, GetRoot().rotation, 0.1f, UltiDraw.DarkRed, UltiDraw.Black);
			UltiDraw.DrawTranslateGizmo(GetRoot().position, GetRoot().rotation, 0.1f);
		}

		if(DrawSkeleton) {
			Action<Bone> recursion = null;
			recursion = new Action<Bone>((bone) => {
				if(bone.GetParent() != null) {
					//if(bone.GetLength() > 0.05f) {
						UltiDraw.DrawBone(
							bone.GetParent().Transform.position,
							Quaternion.FromToRotation(bone.GetParent().Transform.forward, bone.Transform.position - bone.GetParent().Transform.position) * bone.GetParent().Transform.rotation,
							12.5f*BoneSize*bone.GetLength(), bone.GetLength(),
							boneColor.Transparent(alpha)
						);
					//}
				}
				// if(bone.GetName() == "THUMB_CMC_AA" || bone.GetName() == "THUMB_MCP_FE"  || bone.GetName() == "THUMB_IP_FE"){
				// 	UltiDraw.DrawCapsule(
				// 		bone.GetParent().Transform.position * 0.5f + bone.Transform.position * 0.5f,
				// 		Quaternion.FromToRotation(bone.GetParent().Transform.up, bone.Transform.position - bone.GetParent().Transform.position) * bone.GetParent().Transform.rotation,
				// 		// bone.GetParent().Transform.rotation,
				// 		0.2f, bone.GetLength(),
				// 		colliderColor.Transparent(alpha)
				// 	);
				// }
				
				UltiDraw.DrawSphere(
					bone.Transform.position,
					Quaternion.identity,
					10f/5f * BoneSize,
					jointColor.Transparent(alpha)
				);
				for(int i=0; i<bone.Childs.Length; i++) {
					recursion(bone.GetChild(i));
				}
			});
			if(Bones.Length > 0) {
				recursion(Bones[0]);
			}
		}

		if(DrawVelocities) {
			for(int i=0; i<Bones.Length; i++) {
				UltiDraw.DrawArrow(
					Bones[i].Transform.position,
					Bones[i].Transform.position + Bones[i].Velocity,
					0.75f,
					0.0075f,
					0.05f,
					UltiDraw.DarkGreen.Transparent(0.5f)
				);
			}
		}

		if(DrawAccelerations) {
			for(int i=0; i<Bones.Length; i++) {
				UltiDraw.DrawArrow(
					Bones[i].Transform.position,
					Bones[i].Transform.position + Bones[i].Acceleration,
					0.75f,
					0.0075f,
					0.05f,
					UltiDraw.DarkBlue.Transparent(0.5f)
				);
			}
		}

		if(DrawForces) {
			for(int i=0; i<Bones.Length; i++) {
				UltiDraw.DrawArrow(
					Bones[i].Transform.position,
					Bones[i].Transform.position + Bones[i].Force,
					0.75f,
					0.0075f,
					0.05f,
					UltiDraw.DarkRed.Transparent(0.5f)
				);
			}
		}

		if(DrawTransforms) {
			Action<Bone> recursion = null;
			recursion = new Action<Bone>((bone) => {
				UltiDraw.DrawTranslateGizmo(bone.Transform.position, bone.Transform.rotation, 0.1f);
				for(int i=0; i<bone.Childs.Length; i++) {
					recursion(bone.GetChild(i));
				}
			});
			if(Bones.Length > 0) {
				recursion(Bones[0]);
			}
		}
		UltiDraw.End();

		if(DrawHistory) {
			if(DrawSkeleton) {
				for(int i=0; i<History.Count; i++) {
					Sketch(History[i].Transformations, UltiDraw.GetRainbowColor(Index, 2).Transparent(0.5f));
				}
			}
			if(DrawVelocities) {
				float max = 0f;
				List<float[]> functions = new List<float[]>();
				for(int i=0; i<Bones.Length; i++) {
					float[] function = new float[History.Count];
					for(int j=0; j<function.Length; j++) {
						function[j] = History[j].Velocities[i].magnitude;
						max = Mathf.Max(max, function[j]);
					}
					functions.Add(function);
				}
				UltiDraw.Begin();
				UltiDraw.DrawGUIFunctions(new Vector2(0.5f, 0.05f), new Vector2(0.9f, 0.1f), functions, 0f, max, 0.0025f, UltiDraw.DarkGrey, UltiDraw.GetRainbowColors(functions.Count));
				UltiDraw.End();
			}
			if(DrawAccelerations) {
				float max = 0f;
				List<float[]> functions = new List<float[]>();
				for(int i=0; i<Bones.Length; i++) {
					float[] function = new float[History.Count];
					for(int j=0; j<function.Length; j++) {
						function[j] = History[j].Accelerations[i].magnitude;
						max = Mathf.Max(max, function[j]);
					}
					functions.Add(function);
				}
				UltiDraw.Begin();
				UltiDraw.DrawGUIFunctions(new Vector2(0.5f, 0.175f), new Vector2(0.9f, 0.1f), functions, 0f, max, 0.0025f, UltiDraw.DarkGrey, UltiDraw.GetRainbowColors(functions.Count));
				UltiDraw.End();
			}
		}
	}

	public void Sketch(Matrix4x4[] transformations, Color color) {
		if(Bones.Length == 0) {
			return;
		}
		UltiDraw.Begin();
		if(transformations.Length != Bones.Length) {
			for(int i=0; i<transformations.Length; i++) {
				UltiDraw.DrawCube(transformations[i], 0.02f, color);
			}
		} else {
			Action<Bone> recursion = null;
			recursion = new Action<Bone>((bone) => {
				if(bone.GetParent() != null) {
					UltiDraw.DrawLine(transformations[bone.GetParent().Index].GetPosition(), transformations[bone.Index].GetPosition(), color);
				}
				UltiDraw.DrawCube(transformations[bone.Index], 0.02f, color);
				for(int i=0; i<bone.Childs.Length; i++) {
					recursion(bone.GetChild(i));
				}
			});
			recursion(Bones[0]);
		}
		UltiDraw.End();
	}

	void OnRenderObject() {
		Draw();
	}

	void OnDrawGizmos() {
		if(!Application.isPlaying) {
			OnRenderObject();
		}
	}

	public class State {
		public Matrix4x4[] Transformations;
		public Vector3[] Velocities;
		public Vector3[] Accelerations;
	}

	[Serializable]
	public class Bone {
		public Actor Actor;
		public Transform Transform;
		public Vector3 Velocity;
		public Vector3 Acceleration;
		public Vector3 Force;
		public int Index;
		public int Parent;
		public int[] Childs;
		public float Length;
		
		// add after hand, degree of freedom
		public int Dof;

		public Bone(Actor actor, Transform transform, int index) {
			Actor = actor;
			Transform = transform;
			Velocity = Vector3.zero;
			Acceleration = Vector3.zero;
			Index = index;
			Parent = -1;
			Childs = new int[0];
			Length = GetLength();
			Dof = 0;
		}

		public string GetName() {
			return Transform.name;
		}

		public Bone GetParent() {
			return Parent == -1 ? null : Actor.Bones[Parent];
		}

		public Bone GetChild(int index) {
			return index >= Childs.Length ? null : Actor.Bones[Childs[index]];
		}

		public void SetLength(float value) {
			Length = Mathf.Max(float.MinValue, value);
		}

		public float GetLength() {
			return GetParent() == null ? 0f : Vector3.Distance(GetParent().Transform.position, Transform.position);
		}

		public void ComputeLength() {
			Length = GetLength();
		}

		public void ApplyLength() {
			if(GetParent() != null) {
				Transform.position = GetParent().Transform.position + Length * (Transform.position - GetParent().Transform.position).normalized;
			}
		}
	}

	#if UNITY_EDITOR
	[CustomEditor(typeof(Actor))]
	public class Actor_Editor : Editor {

		public Actor Target;

		void Awake() {
			Target = (Actor)target;
		}

		public override void OnInspectorGUI() {
			Undo.RecordObject(Target, Target.name);
	
			Target.Index = EditorGUILayout.IntField("Index", Target.Index);

			Target.DrawRoot = EditorGUILayout.Toggle("Draw Root", Target.DrawRoot);
			Target.DrawSkeleton = EditorGUILayout.Toggle("Draw Skeleton", Target.DrawSkeleton);
			Target.DrawTransforms = EditorGUILayout.Toggle("Draw Transforms", Target.DrawTransforms);
			Target.DrawVelocities = EditorGUILayout.Toggle("Draw Velocities", Target.DrawVelocities);
			Target.DrawAccelerations = EditorGUILayout.Toggle("Draw Accelerations", Target.DrawAccelerations);
			Target.DrawHistory = EditorGUILayout.Toggle("Draw History", Target.DrawHistory);

			Target.MaxHistory = EditorGUILayout.IntField("Max History", Target.MaxHistory);

			Utility.SetGUIColor(Color.grey);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				Utility.ResetGUIColor();
				if(Utility.GUIButton("Skeleton", UltiDraw.DarkGrey, UltiDraw.White)) {
					Target.InspectSkeleton = !Target.InspectSkeleton;
				}
				if(Target.InspectSkeleton) {
					EditorGUILayout.LabelField("Skeleton Bones: " + Target.Bones.Length);
					Target.BoneSize = EditorGUILayout.FloatField("Bone Size", Target.BoneSize);
					Target.JointColor = EditorGUILayout.ColorField("Joint Color", Target.JointColor);
					Target.BoneColor = EditorGUILayout.ColorField("Bone Color", Target.BoneColor);
					InspectSkeleton(Target.GetRoot(), 0);
				}
			}

			if(GUI.changed) {
				EditorUtility.SetDirty(Target);
			}
		}

		private void InspectSkeleton(Transform transform, int indent) {
			Bone bone = Target.FindBone(transform.name);
			Utility.SetGUIColor(bone == null ? UltiDraw.LightGrey : UltiDraw.Mustard);
			using(new EditorGUILayout.HorizontalScope ("Box")) {
				Utility.ResetGUIColor();
				EditorGUILayout.BeginHorizontal();
				for(int i=0; i<indent; i++) {
					EditorGUILayout.LabelField("|", GUILayout.Width(20f));
				}
				EditorGUILayout.LabelField("-", GUILayout.Width(20f));
				EditorGUILayout.LabelField(transform.name + " " + (bone == null ? string.Empty : "(" + bone.Index.ToString() + ")" + " " + "(" + bone.GetLength() + ")"), GUILayout.Width(250f), GUILayout.Height(20f));
				GUILayout.FlexibleSpace();
				if(Utility.GUIButton("Bone", bone == null ? UltiDraw.White : UltiDraw.DarkGrey, bone == null ? UltiDraw.DarkGrey : UltiDraw.White)) {
					Transform[] bones = new Transform[Target.Bones.Length];
					for(int i=0; i<bones.Length; i++) {
						bones[i] = Target.Bones[i].Transform;
					}
					if(bone == null) {
						ArrayExtensions.Add(ref bones, transform);
						Target.ExtractSkeleton(bones);
					} else {
						ArrayExtensions.Remove(ref bones, transform);
						Target.ExtractSkeleton(bones);
					}
				}
				EditorGUILayout.EndHorizontal();
			}
			for(int i=0; i<transform.childCount; i++) {
				InspectSkeleton(transform.GetChild(i), indent+1);
			}
		}

	}
	#endif

}
