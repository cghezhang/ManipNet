using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif
using SDFr;

[ExecuteInEditMode]
public class ObjectManager : MonoBehaviour {
	// used in object trajectory to move the object root to the other place( in default object root)
	// public Vector3 meshCenter;
	// public Vector3 meshExtend;
	// mesh collider
	public MeshCollider[] allMeshColliders; // there might be multiple meshColliders if we cut the meshes
	
	// private void Awake() {
	// 	// every time when awake, need to check if mesh collider is there
	// 	InitMeshColliders();	
	// }

	public Vector3 localCenterofMass = Vector3.zero; // this is center of mass in the object local sapce
	public Matrix4x4 restInertiaMatrix = Matrix4x4.zero; // init with zero


	// ====================================== get SDF======================================= 
    public CheckSDF GetCheckSDF(){
        CheckSDF checkSDF = transform.GetComponent<CheckSDF>();
        if(checkSDF == null){
            Debug.Log("Need to add CheckSDF for " + transform.name);
        }
        return checkSDF;
    }

	public Matrix4x4 GetCubeLocalTransformation(Axis mirrorAxis){
		if(GetCheckSDF()==null){
			return Matrix4x4.identity;
		}
		return GetCheckSDF().GetCubeLocalTransformation(mirrorAxis);
	}

	public Vector3 GetCubeExtend(){
		if(GetCheckSDF()==null){
			return Vector3.zero;
		}
		return GetCheckSDF().GetBounds().size;
	}
	public Vector3 GetMaxCubeExtend(){
		if(GetCheckSDF()==null){
			return Vector3.zero;
		}
		Vector3 size =  GetCheckSDF().GetBounds().size;
		return Vector3.one * Mathf.Max(size.x, size.y, size.z);;
	}
	// ====================================== get SDF======================================= 

	// ====================================== for physics parameter ======================================= 
	public void ExtractInertiaMatrix(){
		// add rigid body
		Rigidbody rig = this.GetComponentInChildren<Rigidbody>();
		if(rig == null){
			rig = this.gameObject.AddComponent<Rigidbody>();
		}
		if(this.transform.position != Vector3.zero || this.transform.rotation != Quaternion.identity){
			Debug.Log("need the rest pose when extract the inertia matrix");
			this.transform.position = Vector3.zero;
			this.transform.rotation = Quaternion.identity;
		}
		
		// calculate the inertia tensor
		rig.isKinematic = false;
		rig.ResetInertiaTensor();
		Vector3 inertiaTensor = rig.inertiaTensor;
		Quaternion inertiaRotation = rig.inertiaTensorRotation;
		Debug.Log("initial Inertia Tensor: " + inertiaTensor.x + " " + inertiaTensor.y + " " + inertiaTensor.z);
		Debug.Log("initial Inertia Rotation: " + inertiaRotation.x + " " + inertiaRotation.y + " " + inertiaRotation.z + " " + inertiaRotation.z);

		// get the rest inertia matrix
		Matrix4x4 ineriaMatrix = Matrix4x4.zero;
		ineriaMatrix[0,0] = inertiaTensor.x;
		ineriaMatrix[1,1] = inertiaTensor.y;
		ineriaMatrix[2,2] = inertiaTensor.z;
		Matrix4x4 rotationMatrix = Matrix4x4.TRS(Vector3.zero, inertiaRotation, Vector3.one);
		restInertiaMatrix = rotationMatrix * ineriaMatrix * rotationMatrix.transpose;
		Debug.Log(restInertiaMatrix);

		// we by the way extract the center of mass, which is relative to the object origin
		localCenterofMass = rig.centerOfMass;
		// remove the rigid body
		DestroyImmediate(rig);
	} 

	public Vector3 GetGlobalCenterOfMass(){
		// todo not sure if work with mirror
		if(restInertiaMatrix==Matrix4x4.zero){
			Debug.LogError("need to initialize the center of mass firstly");
			return Vector3.zero;
		}
		return localCenterofMass.GetRelativePositionFrom(this.transform.GetWorldMatrix());
	}

	public Matrix4x4 GetRestInertiaMatrix(){
		// todo not sure if work with mirror
		if(restInertiaMatrix==Matrix4x4.zero){
			Debug.LogError("need to initialize the InertiaMatrix firstly");
		}
		return restInertiaMatrix;
	}

	// ====================================== for physics parameter ======================================= 

	// ================= extract center and extends from colliders ============================= 
	// public Vector3 GetMaxExtend(){
	// 	return Vector3.one * Mathf.Max(meshExtend.x, meshExtend.y, meshExtend.z);;
	// }
	// public Matrix4x4 GetMeshCenter(){
	// 	Matrix4x4 m = transform.GetWorldMatrix();
	// 	Vector3 offset = meshCenter.GetRelativePositionFrom(m);
	// 	m[0,3] = offset.x;
	// 	m[1,3] = offset.y;
	// 	m[2,3] = offset.z;
	// 	return m;
	// }

	// public void ExtractCenterExtendFromMeshes(GameObject meshObject){
	// 	MeshCollider meshCollider = meshObject.AddComponent<MeshCollider>();
	// 	meshCollider.convex = true;
	// 	Vector3 centerPosition = meshCollider.bounds.center;
	// 	meshCenter = centerPosition.GetRelativePositionTo(meshObject.transform.parent.GetWorldMatrix()); // need to get relative to original root
	// 	meshExtend = meshCollider.bounds.max-meshCollider.bounds.min;
	// 	DestroyImmediate(meshCollider);
	// }
	// public void ExtractCenterExtendFromMeshes(MeshCollider[] meshColliders){
	// 	if(meshColliders.Length==0){
	// 		ExtractCenterExtendFromMeshes(transform.Find(HandObjectMotionParser.objectMeshName).gameObject);
	// 	}
	// 	else{
	// 		Vector3 centerPoint = Vector3.zero;
	// 		Vector3 extendMax = Vector3.zero;
	// 		Vector3 extendMin = Vector3.zero;
	// 		for(int i=0; i<meshColliders.Length; i++){
	// 			centerPoint += meshColliders[i].bounds.center/meshColliders.Length;
	// 			if(i==0){
	// 				extendMax = meshColliders[i].bounds.max;
	// 				extendMin = meshColliders[i].bounds.min;
	// 			}
	// 			else{
	// 				extendMax.x = Mathf.Max(extendMax.x, meshColliders[i].bounds.max.x);
	// 				extendMax.y = Mathf.Max(extendMax.y, meshColliders[i].bounds.max.y);
	// 				extendMax.z = Mathf.Max(extendMax.z, meshColliders[i].bounds.max.z);
	// 				extendMin.x = Mathf.Min(extendMin.x, meshColliders[i].bounds.min.x);
	// 				extendMin.y = Mathf.Min(extendMin.y, meshColliders[i].bounds.min.y);
	// 				extendMin.z = Mathf.Min(extendMin.z, meshColliders[i].bounds.min.z);
	// 			}
	// 		}
	// 		meshExtend = extendMax-extendMin; // length width height
	// 		// meshCenter = centerPoint;
	// 		meshCenter = centerPoint.GetRelativePositionTo(transform.GetWorldMatrix()); // get relative to the original root 
	// 	}
	// }
	//================= extract center and extends from colliders ============================= 


	//================= collider management ============================= 
	// public MeshCollider[] GetMeshColliders(){
	// 	if(meshColliders==null || meshColliders.Length==0){
	// 		InitMeshColliders();
	// 	}
	// 	return meshColliders;
	// }
	public void AddSimpleMeshCollider(){
		GameObject obj = transform.Find(HandObjectMotionParser.objectMeshName).gameObject;
		MeshCollider meshCollider = obj.GetComponent<MeshCollider>();
		if(meshCollider==null){	
			meshCollider = obj.AddComponent<MeshCollider>();
			meshCollider.convex = true;
			Debug.Log("Added a simple mesh collider for" + this.name);
		} 
		if(allMeshColliders == null || allMeshColliders.Length==0){
			allMeshColliders = new MeshCollider[1];
			allMeshColliders[0] = meshCollider;
			Debug.Log("Refreshed the allMeshColliders");
		}
	}
	//================= collider management ============================= 

	
	#if UNITY_EDITOR
	[CustomEditor(typeof(ObjectManager))]
	public class Interaction_Editor : Editor {
		public ObjectManager Target;
		void Awake() {
			Target = (ObjectManager)target;
		}
		public override void OnInspectorGUI() {
			Undo.RecordObject(Target, Target.name); // don't really know why need to record Target which is the
			Utility.SetGUIColor(UltiDraw.Grey);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				Utility.ResetGUIColor();
				EditorGUILayout.BeginHorizontal();
				EditorGUILayout.EndHorizontal();
				if(Utility.GUIButton("Add Simple Mesh Collider", UltiDraw.DarkGrey, UltiDraw.White)) {
					Target.AddSimpleMeshCollider();
				}
				if(Utility.GUIButton("Extract Rest Inertia Matrix", UltiDraw.DarkGrey, UltiDraw.White)) {
					Target.ExtractInertiaMatrix();
				}
				
			}
			if(GUI.changed) {
				EditorUtility.SetDirty(Target);
			}
		}
        
	}
	#endif

}
