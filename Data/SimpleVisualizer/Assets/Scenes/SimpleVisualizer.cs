#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections;
using System.IO;
using System.Collections.Generic;

[ExecuteInEditMode]
public class SimpleVisualizer : MonoBehaviour
{
    public GameObject hands;
    private Actor leftHand;
    private Actor rightHand;

    public GameObject objects;
    private List<GameObject> sceneObjects = new List<GameObject>();

    public string folder; // here can be a full path or a folder name in a default folder
    private string fullPath;

    public int[] tipIndices = new int[] {5,9,13,17,21}; // end effect
    public int BVHFrameRate = 0;
    public int targetFrameRate = 60;
    
    public MotionSequence leftHandSequence = new MotionSequence(0);
    public MotionSequence rightHandSequence = new MotionSequence(0);
    public List<MotionSequence> objectSequences = new List<MotionSequence>();

    // motion play
    private bool Playing = false;
    private int currentFrame = 0;
    private int totalFrame = 0;
    
    // Start is called before the first frame update
    void LoadBVHs(){
        if(!Directory.Exists(folder)){
            // it can be only the folder name, then will check the default path
            fullPath = Application.dataPath + "/BVH/" + folder;
            Debug.Log("haha" + fullPath);
        }
        else{
            fullPath = folder;
        }
        if(Directory.Exists(fullPath)){
            {// check hands and objects
                if(hands == null){
                    hands = GameObject.Find("Hands");
                    if(hands == null){
                        Debug.LogError("Can not find hands");
                        return;
                    }
                }
                if(objects == null){
                    objects = GameObject.Find("Objects");
                    if(objects == null){
                        Debug.LogError("Can not find objects");
                        return;
                    }
                }
                
                GameObject leftHandObj = hands.transform.Find("LeftHand").gameObject;
                GameObject rightHandObj = hands.transform.Find("RightHand").gameObject;
                if(leftHandObj==null || rightHandObj==null){
                    Debug.LogError("Can not find left/right hand");
                    return;
                }
                if((leftHand = leftHandObj.GetComponent<Actor>()) == null){
                    leftHand = leftHandObj.AddComponent<Actor>();
                    leftHand.RemoveBoneFromSkeleton(0); // remove the current root, the root should be at wrist
                }
                if((rightHand = rightHandObj.GetComponent<Actor>()) == null){
                    rightHand = rightHandObj.AddComponent<Actor>();
                    rightHand.RemoveBoneFromSkeleton(0); // remove the current root, the root should be at wrist
                }
            }
            // deactivate all the child objects
            foreach (Transform child in objects.transform)
            {
                child.gameObject.SetActive(false);
            }
            sceneObjects = new List<GameObject>();
            objectSequences = new List<MotionSequence>();

            // load data
            string[] files = Directory.GetFiles(fullPath);
            for(int i=0; i<files.Length; i++){
                if(files[i].Substring(files[i].Length-4) == ".bvh"){ 
                    string subjectName = files[i].Substring(files[i].LastIndexOf("/")+1, (files[i].LastIndexOf(".") - files[i].LastIndexOf("/")- 1));
                    // Debug.Log(subjectName);
                    if(subjectName == "leftHand"){
                        leftHandSequence = LoadBVH(files[i]);
                    }
                    else if(subjectName == "rightHand"){
                        rightHandSequence = LoadBVH(files[i]);
                    }
                    else{
                        Transform obj = objects.transform.Find(subjectName);
                        if(obj!=null){
                            obj.gameObject.SetActive(true);
                            sceneObjects.Add(obj.gameObject);
                            objectSequences.Add(LoadBVH(files[i]));
                        }
                        else
                        {
                            Debug.Log("No object " + subjectName  + " as the child of " + objects.name);
                        }
                    }
                }
               
            }

            { // check frame numbers
                totalFrame = leftHandSequence.frameNum;
                if(totalFrame != rightHandSequence.frameNum){
                    Debug.Log("left right frames are not matching " + " num1:" + totalFrame + " ,num2:" + rightHandSequence.frameNum);
                }
                for(int i=0; i<objectSequences.Count; i++){
                    if(totalFrame != objectSequences[i].frameNum){
                        Debug.Log("hand frames are not matching with object " + sceneObjects[i].name + " num1:" + totalFrame + " ,num2:" + objectSequences[i].frameNum);
                        break;
                    }
                }
            }
        }
        else
        {
            Debug.Log(fullPath + " does not exist");
        }
    }

    private MotionSequence LoadBVH(string file){
        MotionSequence motionSequence = new MotionSequence(0);
        if(File.Exists(file)) {
            string[] lines = System.IO.File.ReadAllLines(file);
            char[] whitespace = new char[] {' '};
            int index = 0;

            //Create Source Data
            Hierarchy hierarchy = new Hierarchy();
            List<Vector3> offsets = new List<Vector3>();
            List<int[]> channels = new List<int[]>();
            List<float[]> motions = new List<float[]>();
            string name = string.Empty;
            string parent = string.Empty;
            Vector3 offset = Vector3.zero;
            int[] channel = null;
            for(index = 0; index<lines.Length; index++) {
                if(lines[index] == "MOTION") {
                    break;
                }
                string[] entries = lines[index].Split(whitespace);
                for(int entry=0; entry<entries.Length; entry++) {
                    if(entries[entry].Contains("ROOT")) {
                        parent = "None";
                        name = entries[entry+1];
                        break;
                    } else if(entries[entry].Contains("JOINT")) {
                        parent = name;
                        name = entries[entry+1];
                        break;
                    } else if(entries[entry].Contains("End")) {
                        parent = name;
                        name = name+entries[entry+1];
                        string[] subEntries = lines[index+2].Split(whitespace);
                        for(int subEntry=0; subEntry<subEntries.Length; subEntry++) {
                            if(subEntries[subEntry].Contains("OFFSET")) {
                                offset.x = FileUtility.ReadFloat(subEntries[subEntry+1]);
                                offset.y = FileUtility.ReadFloat(subEntries[subEntry+2]);
                                offset.z = FileUtility.ReadFloat(subEntries[subEntry+3]);
                                break;
                            }
                        }
                        hierarchy.AddBone(name, parent, 0);
                        offsets.Add(offset);
                        channels.Add(new int[0]);
                        index += 2;
                        break;
                    } else if(entries[entry].Contains("OFFSET")) {
                        offset.x = FileUtility.ReadFloat(entries[entry+1]);
                        offset.y = FileUtility.ReadFloat(entries[entry+2]);
                        offset.z = FileUtility.ReadFloat(entries[entry+3]);
                        break;
                    } else if(entries[entry].Contains("CHANNELS")) {
                        channel = new int[FileUtility.ReadInt(entries[entry+1])];
                        for(int i=0; i<channel.Length; i++) {
                            if(entries[entry+2+i] == "Xposition") {
                                channel[i] = 1;
                            } else if(entries[entry+2+i] == "Yposition") {
                                channel[i] = 2;
                            } else if(entries[entry+2+i] == "Zposition") {
                                channel[i] = 3;
                            } else if(entries[entry+2+i] == "Xrotation") {
                                channel[i] = 4;
                            } else if(entries[entry+2+i] == "Yrotation") {
                                channel[i] = 5;
                            } else if(entries[entry+2+i] == "Zrotation") {
                                channel[i] = 6;
                            }
                        }
                        hierarchy.AddBone(name, parent, 0);
                        offsets.Add(offset);
                        channels.Add(channel);
                        break;
                    } else if(entries[entry].Contains("}")) {
                        name = parent;
                        parent = name == "None" ? "None" : hierarchy.FindBone(name).Parent;
                        break;
                    }
                }
            }

            //Set Frames
            index += 1;
            while(lines[index].Length == 0) {
                index += 1;
            }
            //ArrayExtensions.Resize(ref poses, FileUtility.ReadInt(lines[index].Substring(8)));
            motionSequence = new MotionSequence(FileUtility.ReadInt(lines[index].Substring(8)));
            //Set Framerate
            index += 1;
            BVHFrameRate = Mathf.RoundToInt(1f / FileUtility.ReadFloat(lines[index].Substring(12)));

            //Compute Frames
            index += 1;
            for(int i=index; i<lines.Length; i++) {
                motions.Add(FileUtility.ReadArray(lines[i]));
            }
            for(int k=0; k<motionSequence.frameNum; k++) {
                motionSequence.poses[k] = new Pose(hierarchy.Bones.Length);
                int idx = 0;
                for(int i=0; i<hierarchy.Bones.Length; i++) {
                    Hierarchy.Bone info = hierarchy.Bones[i];
                    Vector3 position = Vector3.zero;
                    Quaternion rotation = Quaternion.identity;
                    for(int j=0; j<channels[i].Length; j++) {
                        if(channels[i][j] == 1) {
                            position.x = motions[k][idx]; idx += 1;
                        }
                        if(channels[i][j] == 2) {
                            position.y = motions[k][idx]; idx += 1;
                        }
                        if(channels[i][j] == 3) {
                            position.z = motions[k][idx]; idx += 1;
                        }
                        if(channels[i][j] == 4) {
                            rotation *= Quaternion.AngleAxis(motions[k][idx], Vector3.right); idx += 1;
                        }
                        if(channels[i][j] == 5) {
                            rotation *= Quaternion.AngleAxis(motions[k][idx], Vector3.up); idx += 1;
                        }
                        if(channels[i][j] == 6) {
                            rotation *= Quaternion.AngleAxis(motions[k][idx], Vector3.forward); idx += 1;
                        }
                    }
                    position = (position == Vector3.zero ? offsets[i] : position);
                    Matrix4x4 local = Matrix4x4.TRS(position, rotation, Vector3.one);
                    motionSequence.poses[k].world[i] = info.Parent == "None" ? local : motionSequence.poses[k].world[hierarchy.FindBone(info.Parent).Index] * local;
                }
            }
            Debug.Log(file + "is loaded");
        } 
        else
        {
            Debug.Log(file + "can not be found");
        }
        return motionSequence;
    }


    // [System.Serializable]
    public class MotionSequence{
        public int frameNum; // individual numbers
        public Pose[] poses;
        public MotionSequence(int frameNumer){
            if(frameNum>=0){
                frameNum = frameNumer;
                poses = new Pose[frameNum];
            }
        }
    }
    // [System.Serializable]
    public class Pose {
        public int jointNum; 
        public Matrix4x4[] world;
        public Pose(int jointNumber){
            if(jointNumber>0){
                jointNum = jointNumber;
                world = new Matrix4x4[jointNumber];
            }
        }
	}

	public class Hierarchy {
		public Bone[] Bones;
		private string[] Names = null;
		public Hierarchy() {
			Bones = new Bone[0];
		}
		public void AddBone(string name, string parent, int dof) {
			ArrayExtensions.Add(ref Bones, new Bone(Bones.Length, name, parent, dof));
		}
		public Bone FindBone(string name) {
			return System.Array.Find(Bones, x => x.Name == name);
		}
		public string[] GetBoneNames() {
			if(Names == null || Names.Length != Bones.Length) {
				Names = new string[Bones.Length];
				for(int i=0; i<Bones.Length; i++) {
					Names[i] = Bones[i].Name;
				}
			}
			return Names;
		}
		public class Bone {
			public int Index = -1;
			public string Name = "";
			public string Parent = "";
			public float Mass = 1f;
			public Vector3 Alignment = Vector3.zero;
			public int Dof = 0;
			public Bone(int index, string name, string parent, int dof) {
				Index = index;
				Name = name;
				Parent = parent;
				Mass = 1f;
				Alignment = Vector3.zero;
				Dof = dof;
			}
		}
	}
    private void LoadFrame(int frameIndex){
        // left hand
        int ignoreNum = 0;
        for(int i=0; i<leftHandSequence.poses[frameIndex].world.Length; i++){
            if(tipIndices.Contains(i)){
                ignoreNum++;
                // Debug.Log(i);
            }
            else
            {
                leftHand.Bones[i-ignoreNum].Transform.position = leftHandSequence.poses[frameIndex].world[i].GetPosition();
                leftHand.Bones[i-ignoreNum].Transform.rotation = leftHandSequence.poses[frameIndex].world[i].GetRotation();
            }
        }
        // right hand
        ignoreNum = 0;
        for(int i=0; i<rightHandSequence.poses[frameIndex].world.Length; i++){
            if(tipIndices.Contains(i)){
                ignoreNum++;
            }
            else
            {
                rightHand.Bones[i-ignoreNum].Transform.position = rightHandSequence.poses[frameIndex].world[i].GetPosition();
                rightHand.Bones[i-ignoreNum].Transform.rotation = rightHandSequence.poses[frameIndex].world[i].GetRotation();
            }
        }
        // object
        for(int iObj=0; iObj<sceneObjects.Count; iObj++){
            sceneObjects[iObj].transform.position = objectSequences[iObj].poses[frameIndex].world[0].GetPosition();
            sceneObjects[iObj].transform.rotation = objectSequences[iObj].poses[frameIndex].world[0].GetRotation();
        }
    }

    private IEnumerator Play() {
        System.DateTime previous = Utility.GetTimestamp();
        while(leftHandSequence.frameNum>0 || rightHandSequence.frameNum>0 || (objectSequences.Count>0 && objectSequences[0].frameNum>0)){
            while(Utility.GetElapsedTime(previous) < 1f/targetFrameRate) {
                yield return new WaitForSeconds(0f);
            }
            System.DateTime current = Utility.GetTimestamp();
            currentFrame = currentFrame >= totalFrame-1? 0 : currentFrame+(120/targetFrameRate);
            // Debug.Log(currentFrame);
            {
                LoadFrame(currentFrame);
            }
            previous = current;
        }
	}

    public void PlayAnimation() {
        if(Playing) {
			return;
		}
		Playing = true;
		EditorCoroutines.StartCoroutine(Play(), this);
	}

    public void StopAnimation() {
        if(!Playing) {
			return;
		}
		Playing = false;
		EditorCoroutines.StopCoroutine(Play(), this);
	}

    [CustomEditor(typeof(SimpleVisualizer))]
	public class Visualizer_Editor : Editor {
        private SimpleVisualizer myTarget;
        private bool copyMotion;
        void Awake() {
                myTarget = (SimpleVisualizer)target;
            }
        public override void OnInspectorGUI() {
			Inspector();
			if(GUI.changed) {
				EditorUtility.SetDirty(myTarget);
			}
            if(myTarget.Playing){
                Repaint();
            }
		}
        
        // Editor
        public void Inspector() {
            DrawDefaultInspector();
            if(Utility.GUIButton("Load", UltiDraw.DarkGrey, UltiDraw.White)) {
                myTarget.LoadBVHs();
                myTarget.currentFrame = 0;
                myTarget.LoadFrame(myTarget.currentFrame);
            }
            if(myTarget.leftHandSequence.frameNum>0 || myTarget.rightHandSequence.frameNum>0 || 
              (myTarget.objectSequences.Count>0 && myTarget.objectSequences[0].frameNum>0)){
                if(myTarget.Playing) {
                    if(Utility.GUIButton("||", Color.red, Color.black, 50f, 20f)) {
                        myTarget.StopAnimation();
                    }
                } else {
                    if(Utility.GUIButton("|>", Color.green, Color.black, 50f, 20f)) {
                        myTarget.PlayAnimation();
                    }
                }
                int index = EditorGUILayout.IntSlider(myTarget.currentFrame, 0, myTarget.totalFrame-1);
                if(index != myTarget.currentFrame) {
                    myTarget.currentFrame = index;
                    myTarget.LoadFrame(myTarget.currentFrame);
                }
            }
            else
            {
                EditorGUILayout.TextField("Please load the motions");
            }
        }
    }
}
#endif