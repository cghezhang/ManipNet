using UnityEngine;
using SDFr;

[System.Serializable]
public class ObjectColliderBuilder
{   
    // // for voxel colliders
    // public static string InteractionLayerName = "Interaction";
    // // for mesh colliders
    // public static string IKLayerName = "IK";
    
    // this exist only if there is runtimeObj
    public GameObject runtimeObj;
    
    // public MotionData Data;
    public string objectName;
    public CuboidMap cuboidMap;

    // public ObjectColliderBuilder(MotionData data, string objName){
    //     Data = data;
    //     objectName = objName;
    // }

    public ObjectColliderBuilder(GameObject obj){
        runtimeObj = obj;
    }

    public GameObject GetObject(){
        if(runtimeObj){
            return runtimeObj;
        }
        // if(Data != null && objectName != null){
        //     foreach(GameObject instance in Data.GetScene().GetRootGameObjects()) {
        //         if(instance.transform.Find(objectName)){
        //             return instance.transform.Find(objectName).gameObject;
        //         }
        //     }
        // }
        Debug.LogError("Can't find the object for building colliders");
        return null;
    }

    public ObjectManager GetObjectManager(){
        GameObject obj;
        if(runtimeObj == null){ 
            obj = GetObject();
        }
        else{
            obj = runtimeObj;
        }
        ObjectManager objectManager = obj.GetComponent<ObjectManager>();
        if(objectManager == null && obj != null){
            Debug.Log("Need to add object manager for " + obj.name);
        }
        else if(objectManager.allMeshColliders==null || objectManager.allMeshColliders.Length==0){
            Debug.Log("Need to add mesh colliders for " + obj.name);
        }
        return objectManager;
    }

    public Vector3 GetGlobalCenterOfMass(){
        // todo not sure if work with mirror
        return GetObjectManager().GetGlobalCenterOfMass();
    }
    public Matrix4x4 GetRestInertiaMatrix(){
        // todo not sure if work with mirror
        return GetObjectManager().GetRestInertiaMatrix();
    }

    //========= sense the mesh collider ==========
    // public CuboidMap UpdateCuboidMap(int propResolution = 10) {
    //     cuboidMap = new CuboidMap(new Vector3Int(propResolution, propResolution, propResolution));
    //     ObjectManager objectManager = GetObjectManager();
    //     if(objectManager != null){
    //         //cuboidMap.Sense(objectManager.GetCubeWorldTransformation(), LayerMask.GetMask(HandObjectMotionParser.InteractionLayerName), objectManager.GetCubeExtend(), 0f, false);
    //         cuboidMap.Sense(objectManager.GetCubeWorldTransformation(), LayerMask.GetMask(HandObjectMotionParser.InteractionLayerName), objectManager.GetMaxCubeExtend(), 0f, false);
    //     }
    //     else{
    //         Debug.Log("Object Manager is missed");
    //     }
    //     return cuboidMap;
    // }

    // public void DrawContact(Color free, Color colission, float threshold){
    //     cuboidMap.DrawContact(free, colission, threshold);
    // }
    //========= sense the mesh collider ==========

    public CheckSDF GetCheckSDF(){
        GameObject obj;
        if(runtimeObj == null){ 
            obj = GetObject();
        }
        else{
            obj = runtimeObj;
        }
        CheckSDF checkSDF = obj.GetComponent<CheckSDF>();
        if(checkSDF == null && obj != null){
            Debug.Log("Need to add CheckSDF for " + obj.name);
        }
        return checkSDF;
    }

    
    public CuboidMap UpdateCuboidMap() {
        CheckSDF checkSDF = GetCheckSDF();
        cuboidMap = checkSDF.GetCuboidMap();
        if(cuboidMap != null){
            /**
            CubeWorld/CubeLocal Transformation is the center of the SDF bounding box, 
            which is used in pivot of cuboid and applying the offset for timeseries trajectory
            */
            cuboidMap.Retransform(checkSDF.GetCubeWorldTransformation(), checkSDF.GetCurrentMirrorAxis()); 
        }
        else{
            Debug.Log("cuboid map is null in checkSDF");
        }
        return cuboidMap;
    }
    public float[] GetRealDistanceValues(){
        CheckSDF checkSDF = GetCheckSDF();
        if(checkSDF!=null && checkSDF.GetCuboidMap()!= null){
            return checkSDF.GetRealDistances();
        }
        Debug.Log("no CheckSDF or SDF is not initialized");
        float[] dis = new float[0];
        return dis;
    }

    public void DrawContact(Color free, Color colission, float threshold){
        cuboidMap.DrawVoxelByDistance(free, 0f);
        // GetRealDistanceValues().Print();
    }
    public void DrawContact(Color free){
        cuboidMap.DrawVoxelByDistance(free, 0f);
        // GetRealDistanceValues().Print();
    }

    // Mesh Collider
    public MeshCollider GetMeshCollider(){
        GameObject obj;
        if(runtimeObj == null){ 
            obj = GetObject();
        }
        else{
            obj = runtimeObj;
        }
        MeshCollider meshCollider = obj.GetComponentInChildren<MeshCollider>();
        return meshCollider;
    }

    public MeshCollider[] GetMeshColliders(){
        GameObject obj;
        if(runtimeObj == null){ 
            obj = GetObject();
        }
        else{
            obj = runtimeObj;
        }
        MeshCollider[] meshColliders = obj.GetComponentsInChildren<MeshCollider>();
        return meshColliders;
    }

}