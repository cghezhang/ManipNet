using UnityEngine;
using System.IO;
using System.Collections.Generic;

public class CSVParser
{ 
    /*
    This parser can read the input trajectories from TXT file and CSV file
    Data stream from Vicon software (e.g. Tracker) can also be organized similarly
    */
    // data 
    public TrajectoryData trajData = null;
    // alignemnt 
    public Matrix4x4[] handAlignmentTranses = null; 
    public Matrix4x4[] objectAlignmentTranses = null;

    public TrajectorySegments InisializeSegments(){
        if(trajData!=null){
            return new TrajectorySegments(trajData);
        }
        Debug.Log("have no source data, need to read files firstly");
        return null;
    }

    public void AnalyzeFiles(string handObjectPath, string handAlignmentPath, string objAlignmentPath){
        // add suffix 'CSV'
        handObjectPath = handObjectPath + ".csv";
        handAlignmentPath = handAlignmentPath + ".csv";
        objAlignmentPath = objAlignmentPath + ".csv";
        // read the hand object trajectory
        if(File.Exists(handObjectPath) && File.Exists(handAlignmentPath) && File.Exists(objAlignmentPath)){
            trajData = ReadFile(handObjectPath);
        }else{
            Debug.LogError("file path is not available " + handObjectPath + " " + handAlignmentPath + " " + objAlignmentPath);
        }
        AnalyzeAlignmentFiles(handAlignmentPath, objAlignmentPath);
    }
    public void AnalyzeAlignmentFiles(string handAlignmentPath, string objAlignmentPath){
        /// <summary>
        /// note here, we will put the object at the origin and left right hand as the rest pose (but need to give an additional world rotation)
        /// need to take care of the offset like the height of the object and wrist offset (because the marker board may not exactly locate at the wrist end)
        /// <summary>
        handAlignmentTranses = hardCodeHandsAlignment(handAlignmentPath);
        objectAlignmentTranses = hardCodeObjectAlignment(objAlignmentPath);
    }

    public void AnalyzeTXTFiles(string filePath){
        /// <summary>
        /// for txt file we exported from Editor
        /// </summary>
        if(File.Exists(filePath)){
            trajData = ReadTXTFile(filePath);
        }else{
            Debug.LogError("txt file path is not available");
        }
    }
    private TrajectoryData ReadTXTFile(string fileName){
        TrajectoryData data = null;
        // hard code some statistics
        int headLength = 1;
        int numTrans = 3; // order is obj, leftHand and rightHand

        //record the axis angles and positions
        // quaternion, order is XYZW
        float quaterX = 0f;
        float quaterY = 0f;
        float quaterZ = 0f;
        float quaterW = 0f;
        //position
        float posX = 0f;
        float posY = 0f;
        float posZ = 0f;

        string[] lines = System.IO.File.ReadAllLines(fileName);
        char[] whitespace = new char[]{' ', ','};
        // frame number
        int frameNum = lines.Length;
        while(lines[frameNum-1] == string.Empty){
            frameNum -= 1;
        }
        frameNum = frameNum-headLength;
        Debug.Log("total frames: " + frameNum);
        // frame rate
        string[] entries = System.Text.RegularExpressions.Regex.Replace(lines[0], @"\s+", " ").Split(whitespace);
        int frameRate = FileUtility.ReadInt(entries[0]);
        Debug.Log("frame rate: " + frameRate);
        // initialize Data
        data = new TrajectoryData(this, frameNum, frameRate);
        // read transformations
        for(int iFrame=0; iFrame<frameNum; iFrame++){
            // read lines
            entries = System.Text.RegularExpressions.Regex.Replace(lines[iFrame+headLength], @"\s+", " ").Split(whitespace);
            if(entries.Length == 7*numTrans){
                int index_value = 0; 
                { // object    
                    quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                    quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                    quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                    quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                    posX = FileUtility.ReadFloat(entries[index_value+4]);
                    posY = FileUtility.ReadFloat(entries[index_value+5]);
                    posZ = FileUtility.ReadFloat(entries[index_value+6]);
                    data.objTraj[iFrame] = Matrix4x4.TRS(new Vector3(posX, posY, posZ), new Quaternion(quaterX, quaterY, quaterZ, quaterW), Vector3.one);
                }
                { // left    
                    index_value += 7;
                    quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                    quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                    quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                    quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                    posX = FileUtility.ReadFloat(entries[index_value+4]);
                    posY = FileUtility.ReadFloat(entries[index_value+5]);
                    posZ = FileUtility.ReadFloat(entries[index_value+6]);
                    data.leftWristTraj[iFrame] = Matrix4x4.TRS(new Vector3(posX, posY, posZ), new Quaternion(quaterX, quaterY, quaterZ, quaterW), Vector3.one);
                }
                { // right   
                    index_value += 7; 
                    quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                    quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                    quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                    quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                    posX = FileUtility.ReadFloat(entries[index_value+4]);
                    posY = FileUtility.ReadFloat(entries[index_value+5]);
                    posZ = FileUtility.ReadFloat(entries[index_value+6]);
                    data.rightWristTraj[iFrame] = Matrix4x4.TRS(new Vector3(posX, posY, posZ), new Quaternion(quaterX, quaterY, quaterZ, quaterW), Vector3.one);
                }
            }
            else{
                Debug.LogError("Data length is wrong in the frame of " + (iFrame+1));
            }
        }
        Debug.Log("hand-object TXT is read");
        return data;
    }

    private TrajectoryData ReadFile (string fileName){
        TrajectoryData data = null;
        // hard code some statistics
        int headLength = 5;
        int stringSkipLen = 2;
        int numTrans = 3; // order is obj, leftHand and rightHand

        //record the axis angles and positions
        // quaternion, order is XYZW
        float quaterX = 0f;
        float quaterY = 0f;
        float quaterZ = 0f;
        float quaterW = 0f;
        //position is mm, so need to scale down mm -> dm
        float posX = 0f;
        float posY = 0f;
        float posZ = 0f;

        string[] lines = System.IO.File.ReadAllLines(fileName);
        char[] whitespace = new char[]{' ', ','};
        // frame number
        int frameNum = lines.Length;
        while(lines[frameNum-1] == string.Empty){
            frameNum -= 1;
        }
        frameNum = frameNum-headLength;
        Debug.Log("total frames: " + frameNum);
        // frame rate
        string[] entries = System.Text.RegularExpressions.Regex.Replace(lines[1], @"\s+", " ").Split(whitespace);
        int frameRate = FileUtility.ReadInt(entries[0]);
        // initialize Data
        data = new TrajectoryData(this, frameNum, frameRate);
        // read transformations
        for(int iFrame=0; iFrame<frameNum; iFrame++){
            // read lines
            entries = System.Text.RegularExpressions.Regex.Replace(lines[iFrame+headLength], @"\s+", " ").Split(whitespace);
            if(entries.Length == 6*numTrans+stringSkipLen){ // this is helical axis angle but do not know how to with this
                Debug.Log("no implementation for the helical axis angle so far");
            }
            else if(entries.Length == 7*numTrans+stringSkipLen){
                // start from the third value of the row (ignore the first 2 values)
                int index_value = stringSkipLen; 
                
                { // object    
                    quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                    quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                    quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                    quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                    posX = FileUtility.ReadFloat(entries[index_value+4]);
                    posY = FileUtility.ReadFloat(entries[index_value+5]);
                    posZ = FileUtility.ReadFloat(entries[index_value+6]);
                    data.objTraj[iFrame] = hardCodeViconUnityMapping(quaterX, quaterY, quaterZ, quaterW, posX, posY, posZ);
                }
                { // left    
                    index_value += 7;
                    quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                    quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                    quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                    quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                    posX = FileUtility.ReadFloat(entries[index_value+4]);
                    posY = FileUtility.ReadFloat(entries[index_value+5]);
                    posZ = FileUtility.ReadFloat(entries[index_value+6]);
                    data.leftWristTraj[iFrame] = hardCodeViconUnityMapping(quaterX, quaterY, quaterZ, quaterW, posX, posY, posZ);
                }
                { // right   
                    index_value += 7; 
                    quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                    quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                    quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                    quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                    posX = FileUtility.ReadFloat(entries[index_value+4]);
                    posY = FileUtility.ReadFloat(entries[index_value+5]);
                    posZ = FileUtility.ReadFloat(entries[index_value+6]);
                    data.rightWristTraj[iFrame] = hardCodeViconUnityMapping(quaterX, quaterY, quaterZ, quaterW, posX, posY, posZ);
                }
            }
            else{
                Debug.LogError("Data length is wrong in the frame of " + (iFrame+1));
            }
        }
        Debug.Log("hand-object CSV is read");
        return data;
	}

    public static Matrix4x4 hardCodeViconUnityMapping(float quaterX, float quaterY, float quaterZ, float quaterW, float posX, float posY, float posZ){
        // first mirror along the Z axis
        Quaternion rot = new Quaternion(-quaterX, -quaterY, quaterZ, quaterW);
        Vector3 pos = new Vector3(posX, posY, -posZ) * 0.01f;
        Matrix4x4 viconTrans = Matrix4x4.TRS(pos, rot, Vector3.one);
        // then rotate along X axis for 90 degree for the world coordinate;
        Matrix4x4 trans = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(90f, 0f, 0f), Vector3.one);
        viconTrans = trans * viconTrans;    
        return viconTrans;
    }

    private Matrix4x4[] hardCodeObjectAlignment(string objectAlignmentFile){
        // object origin; object locals transformation 
        Matrix4x4[] objectTranses = new Matrix4x4[2]; 

        // read origin transe
        string[] lines = System.IO.File.ReadAllLines(objectAlignmentFile);
        char[] whitespace = new char[]{' ', ','};
        int headLength = 5;
        int stringSkipLen = 2;
        int numTrans = 1;

        //record the axis angles and positions
        // quaternion, order is XYZW
        float quaterX = 0f;
        float quaterY = 0f;
        float quaterZ = 0f;
        float quaterW = 0f;
        //position is mm, so need to scale down mm -> dm
        float posX = 0f;
        float posY = 0f;
        float posZ = 0f;

        string[] entries = System.Text.RegularExpressions.Regex.Replace(lines[headLength], @"\s+", " ").Split(whitespace);
        if(entries.Length == 6*numTrans+stringSkipLen){ // this is helical axis angle but do not know how to with this
            Debug.Log("no implementation for the helical axis angle so far");
        }
        else if(entries.Length == 7*numTrans+stringSkipLen){
            // start from the third value of the row (ignore the first 2 values)
            int index_value = stringSkipLen; 
            { // object    
                quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                posX = FileUtility.ReadFloat(entries[index_value+4]);
                posY = FileUtility.ReadFloat(entries[index_value+5]);
                posZ = FileUtility.ReadFloat(entries[index_value+6]);
                objectTranses[0] = hardCodeViconUnityMapping(quaterX, quaterY, quaterZ, quaterW, posX, posY, posZ);
            }  
        }
        else{
            Debug.LogError("Data length is wrong for object alignment");
        }
        // get the local offset, hard code the offset
        Matrix4x4 targetTrans = Matrix4x4.TRS(new Vector3(0f, 0.3f, 0f), Quaternion.Euler(0f, 0f, 0f), Vector3.one);
        Matrix4x4 localTrans = objectTranses[0].inverse * targetTrans;
        // save the local trans 
        objectTranses[1] = localTrans;
        return objectTranses;
    }

    private Matrix4x4[] hardCodeHandsAlignment(string handsAlignmentFile){
        // left/right originals; left/right locals
        Matrix4x4[] handTranses = new Matrix4x4[4]; 
        
        // read origin transe
        string[] lines = System.IO.File.ReadAllLines(handsAlignmentFile);
        char[] whitespace = new char[]{' ', ','};
        int headLength = 5;
        int stringSkipLen = 2;
        int numTrans = 2;

        //record the axis angles and positions
        // quaternion, order is XYZW
        float quaterX = 0f;
        float quaterY = 0f;
        float quaterZ = 0f;
        float quaterW = 0f;
        //position is mm, so need to scale down mm -> dm
        float posX = 0f;
        float posY = 0f;
        float posZ = 0f;

        string[] entries = System.Text.RegularExpressions.Regex.Replace(lines[headLength], @"\s+", " ").Split(whitespace);
        if(entries.Length == 6*numTrans+stringSkipLen){ // this is helical axis angle but do not know how to with this
            Debug.Log("no implementation for the helical axis angle so far");
        }
        else if(entries.Length == 7*numTrans+stringSkipLen){
            // start from the third value of the row (ignore the first 2 values)
            int index_value = stringSkipLen; 
            { // left    
                quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                posX = FileUtility.ReadFloat(entries[index_value+4]);
                posY = FileUtility.ReadFloat(entries[index_value+5]);
                posZ = FileUtility.ReadFloat(entries[index_value+6]);
                handTranses[0] = hardCodeViconUnityMapping(quaterX, quaterY, quaterZ, quaterW, posX, posY, posZ);
            }  
            { // left    
                index_value += 7;
                quaterX = FileUtility.ReadFloat(entries[index_value+0]);
                quaterY = FileUtility.ReadFloat(entries[index_value+1]);
                quaterZ = FileUtility.ReadFloat(entries[index_value+2]);
                quaterW = FileUtility.ReadFloat(entries[index_value+3]);
                posX = FileUtility.ReadFloat(entries[index_value+4]);
                posY = FileUtility.ReadFloat(entries[index_value+5]);
                posZ = FileUtility.ReadFloat(entries[index_value+6]);
                handTranses[1] = hardCodeViconUnityMapping(quaterX, quaterY, quaterZ, quaterW, posX, posY, posZ);
            }  

            { // get the local trans, hard code a bit
                Matrix4x4 leftTargetTrans = Matrix4x4.TRS(new Vector3(0f, -0.15f, -0.35f), Quaternion.Euler(-90f, 0f, -90f), Vector3.one);
                Matrix4x4 rightTargetTrans = Matrix4x4.TRS(new Vector3(0f, -0.15f, 0.35f), Quaternion.Euler(-90f, 0f, -90f), Vector3.one);
                Matrix4x4 leftLocal = handTranses[0].inverse * leftTargetTrans;
                Matrix4x4 rightLocal = handTranses[1].inverse * rightTargetTrans;
                // save the local
                handTranses[2] = leftLocal;
                handTranses[3] = rightLocal;
            }
        }
        else{
            Debug.LogError("Data length is wrong for object alignment " + entries.Length);
        }
        return handTranses;
    }

	[System.Serializable]
	public class TrajectoryData {
        public CSVParser parser;
		public int frameNum;
		public int frameRate;
        public float totalTime;
		public Matrix4x4[] leftWristTraj;
		public Matrix4x4[] rightWristTraj;
		public Matrix4x4[] objTraj; // this should be the object center transformation

        public TrajectoryData(CSVParser parser_, int frameNum_, int frameRate_){
            parser = parser_;
            frameNum = frameNum_;
            frameRate = frameRate_;
            totalTime = ((float)(frameNum-1))/((float)frameRate); 
            leftWristTraj = new Matrix4x4[frameNum];
            rightWristTraj = new Matrix4x4[frameNum];
            objTraj = new Matrix4x4[frameNum];
        }
        public int GetFrameIndex(float time){
            // given the time step, return corresponding frameIndex
            time = time<0f? 0f:time;
            time = time>totalTime? totalTime:time;
            return (int)(time*frameRate);  
        }
        public float GetTime(int frameIndex){
            // given the frameIndex starting from 0, return corresponding time
            frameIndex = frameIndex<0? 0:frameIndex;
            frameIndex = frameIndex>frameNum-1? frameNum-1:frameIndex;
            return ((float)frameIndex)/((float)frameRate);
        }

        /// <summary>
        ///  get trans might be a bit complex
        ///  1. we wish to get from the time(s)
        ///  2. for the mirror we here use the VISUAL left/right, when mirror the visualLeft is actually right
        ///  3. there is local offset we wish to use to align for the rest pose, need to mirror this when mirroring happen
        /// </summary>
        public Matrix4x4 GetVisualLeftWristTrans(float time, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            if(applyLocalOffset){
                if(mirrorAxis == Axis.None){
                    return leftWristTraj[GetFrameIndex(time)] * parser.handAlignmentTranses[2];
                }
                return (rightWristTraj[GetFrameIndex(time)] * parser.handAlignmentTranses[3]).GetMirror(mirrorAxis);
            }
            if(mirrorAxis == Axis.None){
                return leftWristTraj[GetFrameIndex(time)];
            }
            return rightWristTraj[GetFrameIndex(time)].GetMirror(mirrorAxis);
        }
        public Matrix4x4 GetVisualRightWristTrans(float time, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            if(applyLocalOffset){
                if(mirrorAxis == Axis.None){
                    return rightWristTraj[GetFrameIndex(time)] * parser.handAlignmentTranses[3];
                }
                return (leftWristTraj[GetFrameIndex(time)] * parser.handAlignmentTranses[2]).GetMirror(mirrorAxis);
            }
            if(mirrorAxis == Axis.None){
                return rightWristTraj[GetFrameIndex(time)];
            }
            return leftWristTraj[GetFrameIndex(time)].GetMirror(mirrorAxis);
        }
        public Matrix4x4 GetVisualObjectTrans(float time, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            if(applyLocalOffset){
                return (objTraj[GetFrameIndex(time)] * parser.objectAlignmentTranses[1]).GetMirror(mirrorAxis);
            }
            return objTraj[GetFrameIndex(time)].GetMirror(mirrorAxis);
        }
        public Vector3 GetVisualObjectAxisAngleV_R(float time, float delta, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            // local object axis angle relative to right hand
            Matrix4x4[] visualRightTranses = new Matrix4x4[2];
            Matrix4x4[] visualObjectTranses = new Matrix4x4[2];
            float previousTime = time-delta<0?time+delta:time-delta;
            visualRightTranses[0] = GetVisualRightWristTrans(previousTime, mirrorAxis, applyLocalOffset);
            visualRightTranses[1] = GetVisualRightWristTrans(time, mirrorAxis, applyLocalOffset);
            visualObjectTranses[0] = GetVisualObjectTrans(previousTime, mirrorAxis, applyLocalOffset);
            visualObjectTranses[1] = GetVisualObjectTrans(time, mirrorAxis, applyLocalOffset);
            return Frame.StaticGetObjectRelativeAxisAngleV(visualObjectTranses, visualRightTranses, delta);
        }

        // here we can also query by the index starting from 0
        public Matrix4x4 GetVisualLeftWristTrans(int index, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            return GetVisualLeftWristTrans(GetTime(index), mirrorAxis, applyLocalOffset);
        }
        public Matrix4x4 GetVisualRightWristTrans(int index, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            return GetVisualRightWristTrans(GetTime(index), mirrorAxis, applyLocalOffset);
        }
        public Matrix4x4 GetVisualObjectTrans(int index, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            return GetVisualObjectTrans(GetTime(index), mirrorAxis, applyLocalOffset);
        }
        public Vector3 GetVisualObjectAxisAngleV_R(int index, float delta, Axis mirrorAxis=Axis.None, bool applyLocalOffset=true){
            return GetVisualObjectAxisAngleV_R(GetTime(index), delta, mirrorAxis, applyLocalOffset);
        }

        //=============================================for Vicon real-time=================================================
        public void UpdateTrajectoryData(List<Matrix4x4[]> transformationPool, int frameRate_){
            /// <summary>
            /// this is for receiving the vicon stream
            /// here we hard code the order of trans: obj->left->right
            /// <summary>
            
            if(transformationPool.Count==frameNum && transformationPool[0].Length==3){
                // update frameRate on-fly
                frameRate = frameRate_;
                totalTime = ((float)(frameNum-1))/((float)frameRate); 
                // update transes
                for(int i=0; i<transformationPool.Count; i++){
                    objTraj[i] = transformationPool[i][0];
                    leftWristTraj[i] = transformationPool[i][1];
                    rightWristTraj[i] = transformationPool[i][2];
                }
            }
            else{
                Debug.Log("transformation Pool dim is wrong with : " + transformationPool.Count + "," + transformationPool[0].Length);
            }
        }
        //=============================================for Vicon real-time=================================================
    }

    [System.Serializable]
    public class TrajectorySegments{
        public TrajectoryData sourceData;
        public int segmentLength;
        private Matrix4x4[] visualLeftWristSegment;
        private Matrix4x4[] visualRightWristSegment;
        private Matrix4x4[] visualObjectSegment; // this should be the center of the object
        private Vector3[] visualObjectAxisAngleV_R;

        public TrajectorySegments(TrajectoryData sourceData_){
            sourceData = sourceData_;
        }

        public Matrix4x4[] GetVisualLeftWristSegment(){
            Matrix4x4[] segment = new Matrix4x4[segmentLength];
            for(int i=0; i<segmentLength; i++){
                segment[i] = visualLeftWristSegment[i];
            }
            return segment;
        }
        public Matrix4x4[] GetVisualRightWristSegment(){
            Matrix4x4[] segment = new Matrix4x4[segmentLength];
            for(int i=0; i<segmentLength; i++){
                segment[i] = visualRightWristSegment[i];
            }
            return segment;
        }
        public Matrix4x4[] GetVisualObjectSegment(){
            Matrix4x4[] segment = new Matrix4x4[segmentLength];
            for(int i=0; i<segmentLength; i++){
                segment[i] = visualObjectSegment[i];
            }
            return segment;
        }
        public Vector3[] GetVisualObjectAxisAngleV_R(){
            Vector3[] segment = new Vector3[segmentLength];
            for(int i=0; i<segmentLength; i++){
                segment[i] = visualObjectAxisAngleV_R[i];
            }
            return segment;
        }
        
        public void UpdateSegments(float timeStep, Axis mirrorAxis, int pastKeys, int futureKeys, float pastWindow, float futureWindow, float delta, bool applyAlignmentOffset = true){
            /// <sumarry>
            /// timeStep is current time in second
            /// <sumarry>
            segmentLength = pastKeys+futureKeys+1;
            visualLeftWristSegment = new Matrix4x4[segmentLength];
            visualRightWristSegment = new Matrix4x4[segmentLength];
            visualObjectSegment = new Matrix4x4[segmentLength];
            visualObjectAxisAngleV_R = new Vector3[segmentLength];
            
            // past
            float startTime = timeStep - pastWindow;
            float pastInterval = pastWindow/(float)pastKeys;
            for(int i=0; i<pastKeys; i++){
                float currentTime = startTime + pastInterval * i;
                visualLeftWristSegment[i] = sourceData.GetVisualLeftWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualRightWristSegment[i] = sourceData.GetVisualRightWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectSegment[i] = sourceData.GetVisualObjectTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectAxisAngleV_R[i] = sourceData.GetVisualObjectAxisAngleV_R(currentTime, delta, mirrorAxis, applyAlignmentOffset);
            }
            // current and future 
            float endTime = timeStep + futureWindow;
            float futureInterval = futureWindow/(float)futureKeys;
            for(int i=0; i<futureKeys+1; i++){
                // first one is current
                float currentTime = timeStep + futureInterval * i;
                visualLeftWristSegment[i+pastKeys] = sourceData.GetVisualLeftWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualRightWristSegment[i+pastKeys] = sourceData.GetVisualRightWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectSegment[i+pastKeys] = sourceData.GetVisualObjectTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectAxisAngleV_R[i+pastKeys] = sourceData.GetVisualObjectAxisAngleV_R(currentTime,delta, mirrorAxis, applyAlignmentOffset);
            }
        }

        public void UpdateSegments(float timeStep, Axis mirrorAxis, int pastKeys, int futureKeys, float pastWindow, float futureWindow, float delta, int filterSize, float filterPower, bool applyAlignmentOffset = true){
            UpdateSegments(timeStep, mirrorAxis, pastKeys, futureKeys, pastWindow, futureWindow, delta, applyAlignmentOffset);
            // apply guassian filter here
            //visualLeftWristSegment.GaussianFilter(filterSize, filterPower);
            //visualRightWristSegment.GaussianFilter(filterSize, filterPower);
            //visualObjectSegment.GaussianFilter(filterSize, filterPower);
            //visualObjectAxisAngleV_R.GaussianFilter(filterSize, filterPower);
        }
        
        public void DrawSegments(){
            if(visualLeftWristSegment!=null && visualLeftWristSegment.Length>0){
                int segmentLength = visualLeftWristSegment.Length;
                UltiDraw.Begin();
                //Connections
                for(int i=0; i<segmentLength-1; i++) {
                    UltiDraw.DrawLine(visualObjectSegment[i].GetPosition(), visualObjectSegment[i+1].GetPosition(), 0.01f, UltiDraw.Blue);
                    UltiDraw.DrawLine(visualLeftWristSegment[i].GetPosition(), visualLeftWristSegment[i+1].GetPosition(), 0.01f, UltiDraw.Red);
                    UltiDraw.DrawLine(visualRightWristSegment[i].GetPosition(), visualRightWristSegment[i+1].GetPosition(), 0.01f, UltiDraw.Green);
                }
                //Positions
                for(int i=0; i<segmentLength; i++) {
                    UltiDraw.DrawCircle(visualObjectSegment[i].GetPosition(), 0.025f, UltiDraw.Blue);
                    UltiDraw.DrawCircle(visualLeftWristSegment[i].GetPosition(), 0.025f, UltiDraw.Red);
                    UltiDraw.DrawCircle(visualRightWristSegment[i].GetPosition(), 0.025f, UltiDraw.Green);
                }
                //Directions
                for(int i=0; i<segmentLength; i++) {
                    UltiDraw.DrawLine(visualObjectSegment[i].GetPosition(), visualObjectSegment[i].GetPosition() + 0.2f*visualObjectSegment[i].GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.5f));
                    UltiDraw.DrawLine(visualLeftWristSegment[i].GetPosition(), visualLeftWristSegment[i].GetPosition() + 0.2f*visualLeftWristSegment[i].GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.5f));
                    UltiDraw.DrawLine(visualRightWristSegment[i].GetPosition(), visualRightWristSegment[i].GetPosition() + 0.2f*visualRightWristSegment[i].GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.5f));
                }
                for(int i=0; i<segmentLength; i++) {
                    UltiDraw.DrawLine(visualObjectSegment[i].GetPosition(), visualObjectSegment[i].GetPosition() + 0.2f*visualObjectSegment[i].GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.5f));
                    UltiDraw.DrawLine(visualLeftWristSegment[i].GetPosition(), visualLeftWristSegment[i].GetPosition() + 0.2f*visualLeftWristSegment[i].GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.5f));
                    UltiDraw.DrawLine(visualRightWristSegment[i].GetPosition(), visualRightWristSegment[i].GetPosition() + 0.2f*visualRightWristSegment[i].GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.5f));
                }
                for(int i=0; i<segmentLength; i++) {
                    UltiDraw.DrawLine(visualObjectSegment[i].GetPosition(), visualObjectSegment[i].GetPosition() + 0.2f*visualObjectSegment[i].GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.5f));
                    UltiDraw.DrawLine(visualLeftWristSegment[i].GetPosition(), visualLeftWristSegment[i].GetPosition() + 0.2f*visualLeftWristSegment[i].GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.5f));
                    UltiDraw.DrawLine(visualRightWristSegment[i].GetPosition(), visualRightWristSegment[i].GetPosition() + 0.2f*visualRightWristSegment[i].GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.5f));
                }
                // draw axis angle
                int currentIndex = segmentLength/2;
                UltiDraw.DrawLine(visualObjectSegment[currentIndex].GetPosition(), 
                                  visualObjectSegment[currentIndex].GetPosition() + visualObjectAxisAngleV_R[currentIndex].GetRelativeDirectionFrom(visualRightWristSegment[currentIndex]), 
                                  0.025f, 0f, UltiDraw.Purple.Transparent(1f));
        
                
                UltiDraw.End();
            }
        }

        //=============================================for Vicon real-time=================================================
        public void UpdateRealTimeSegments(Axis mirrorAxis, int pastKeys, int futureKeys, float pastWindow, float futureWindow, float delta, bool applyAlignmentOffset = true){
            /// <sumarry>     
            /// <sumarry>
            segmentLength = pastKeys+futureKeys+1;
            visualLeftWristSegment = new Matrix4x4[segmentLength];
            visualRightWristSegment = new Matrix4x4[segmentLength];
            visualObjectSegment = new Matrix4x4[segmentLength];
            visualObjectAxisAngleV_R = new Vector3[segmentLength];
            
            float totalTime = sourceData.totalTime;
            // past
            float pastStartTime = totalTime-pastWindow-futureWindow;
            float pastInterval = pastWindow/(float)pastKeys;
            for(int i=0; i<pastKeys; i++){
                float currentTime = pastStartTime + pastInterval * i;
                visualLeftWristSegment[i] = sourceData.GetVisualLeftWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualRightWristSegment[i] = sourceData.GetVisualRightWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectSegment[i] = sourceData.GetVisualObjectTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectAxisAngleV_R[i] = sourceData.GetVisualObjectAxisAngleV_R(currentTime, delta, mirrorAxis, applyAlignmentOffset);
            }
            // current and future 
            float middleTime = totalTime-futureWindow;
            float futureInterval = futureWindow/(float)futureKeys;
            for(int i=0; i<futureKeys+1; i++){
                // first one is current
                float currentTime = middleTime + futureInterval * i;
                visualLeftWristSegment[i+pastKeys] = sourceData.GetVisualLeftWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualRightWristSegment[i+pastKeys] = sourceData.GetVisualRightWristTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectSegment[i+pastKeys] = sourceData.GetVisualObjectTrans(currentTime, mirrorAxis, applyAlignmentOffset);
                visualObjectAxisAngleV_R[i+pastKeys] = sourceData.GetVisualObjectAxisAngleV_R(currentTime,delta, mirrorAxis, applyAlignmentOffset);
            }
        }
        //=============================================for Vicon real-time=================================================
    }
 
}
