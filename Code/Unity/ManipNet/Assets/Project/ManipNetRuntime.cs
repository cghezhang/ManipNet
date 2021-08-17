using UnityEngine;
using System.Collections.Generic;
using DeepLearning;
using SDFr;

public class ManipNetRuntime : NeuralAnimation {
    public const int nHand = 2;
    public const int nJoint = 17;
    public const int nCollider = 19;
    // csv/txt files 
    public string txtPath = string.Empty;
    private CSVParser parser = null;
    private CSVParser.TrajectorySegments trajectorySegments = null;
    private string rootPath = string.Empty;
    // frame index
	public int startFrame = 1;
	public int windowsize = 80;
	private int currentStep =1;
	// mirror Axis for one hand coordinate
    private Axis mirrorAxis = Axis.XPositive;
    // actor
    public Actor[] hands; 
    private Vector3[] predictedPositions = new Vector3[0];
    private Quaternion[] predictedRotations = new Quaternion[0];
    // object
    public GameObject replacedObject = null;
    private Matrix4x4 centerLocal = Matrix4x4.identity;
    private Vector3 anotherLocalOffset = Vector3.zero; // this is for a further offset
    private Vector3 anotherLocalRotation = Vector3.zero; // this is for a further rotation 
    // hand colliders
    private HandColliderBuilder[] handColliderBuilders = null;
    private ObjectColliderBuilder objectColliderBuilder = null;
    private bool drawHandColliders = false;
    private bool drawObjectColliders = false;
    private bool drawContact = false;
    // special position
    private bool drawSpecialPositions = false;
    private Vector3[] fingerTipPositions;
	private	Vector3[] palmColliderStartPositions;
	private	Vector3[] palmColliderEndPositions;
    // IK
    private bool IK = true;
    private float IKFurthestDistance = 0.3f;
    private float IKDiscountThreshold = 0.35f;
    private UltimateIK.Model leftIK = null;
    private UltimateIK.Model rightIK = null;
    private float[] leftContactDis = null;
    private float[] rightContactDis = null; 
    private List<Vector3> contactPointsLeft = new List<Vector3>();
    private List<Vector3> contactPointsRight = new List<Vector3>();
    // trajectory
    private int trajectoryPastKeys = 10;
    private int trajectoryFutureKeys = 10;
    private float trajectoryPastWindow = 0.5f;
    private float trajectoryFutureWindow = 0.5f;
    private int trajectoryFilterSize = 10;
    private float trajectoryFilterPower = 1f;
    // ambient sensor 
    private CuboidMap envSensor = null;
    private bool drawEnv = false;
    private Vector3 envSize = new Vector3(1.8f, 1.8f, 1.8f);
    private Vector3Int envResolution = new Vector3Int(10, 10, 10); 
    private Vector3 envOffset = new Vector3(-0.4f, 0f, -0.9f);
    private LayerMask envLayerMask;     
    // future distance 
    private int futureDistanceLength = 5;
    private int futureDistanceZoomOut = 2;
    private float futureDistancePosThreshold = 1f;
    private float futureDistanceNegThreshold = -0.2f;
    // proximity sensor
    private bool drawProjectionRays = false;
    private float projectionHandDirectionValue = 0.9f; 
    private float projectionWristValue = 0.6f;
    private float projectionThumbDivisionValue = 0.3f;
    private float projectionThumbDirectionValue = 0.6f;
    private int[] projectionVertexIndices = new int[0];
    private Vector3[] ttPoints = new Vector3[0];
    private Vector3[] ttDirections = new Vector3[0];
    private float[] ttDistance = new float[0];
    private float ttposThreshold = 0;
    // network inf
	private int inputDim = 0;
	private int outputDim = 0;
    // rest poses will bake the initial rest pose and recover when destroy
    private Matrix4x4[] restTranses;
    // GUI
    private GUIStyle ButtonStyle;
    private enum DEMO {Demo1, Demo2}
	private DEMO demo = DEMO.Demo1;
    private enum DEMOOBJ {Sphere, Cylinder, Cube, TriPrism, Hemi, Tetra, Bunny}
    private DEMOOBJ demoObj = DEMOOBJ.Sphere; 


	protected override void Setup() {
        { // hard code network parameter path
            rootPath = Application.dataPath;
            rootPath = rootPath.Substring(0, rootPath.Substring(0, rootPath.LastIndexOf("/")).LastIndexOf("/") + 1);
            ((ManipNet)NeuralNetwork).Folder =  rootPath + "ManipNetBIN";
        }

        // Init
        envLayerMask = LayerMask.GetMask("Interaction"); 
        currentStep = startFrame;
        // bake the rest pose
        this.BakeRestPoses();
        // initialize HandCollider
        this.InitHandCollider();
        // intialize hand mesh vertices for Projection sensor
        this.InitProjectionVertices();
        // initialize network input and output dimension
        this.InitNetwork();
        
        InitDemo1();
	}
    private void BakeRestPoses(){
        restTranses = new Matrix4x4[nHand*nJoint];
        for(int iHand=0; iHand<nHand; iHand++){
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                restTranses[iHand*nJoint+iJoint]=hands[iHand].Bones[iJoint].Transform.GetWorldMatrix();
            }
        }
    }
    private Matrix4x4[] GetRestTranses(int handIndex){
		Matrix4x4[] restTrans=new Matrix4x4[nJoint];
		for(int i=0; i<nJoint; i++){
			restTrans[i] = restTranses[handIndex*nJoint+i];
		}
		return restTrans;
	}
    private void ReloadRestPoses(){
        for(int iHand=0; iHand<nHand; iHand++){
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                hands[iHand].Bones[iJoint].Transform.position=restTranses[iHand*nJoint+iJoint].GetPosition();
                hands[iHand].Bones[iJoint].Transform.rotation=restTranses[iHand*nJoint+iJoint].GetRotation();
            }
        }
        runtimeFingerColliderUpdate();
    }
    private void InitCSVTrajectories(){
        parser = new CSVParser();
        // here load txt file and intialize trajectories
        parser.AnalyzeTXTFiles(txtPath);
        trajectorySegments = parser.InisializeSegments();
    }
    private void InitHandCollider(){
        if(hands.Length>0){
            if(handColliderBuilders==null){
                GameObject leftHandSkin = GameObject.Find("/Hands_Two/" + "LeftHand" +"Skin");  
                GameObject rightHandSkin = GameObject.Find("/Hands_Two/" + "RightHand" + "Skin");
                if(leftHandSkin!=null && rightHandSkin!=null){
                    MeshFilter leftHandMesh = leftHandSkin.GetComponent<MeshFilter>();
                    MeshFilter rightHandMesh = rightHandSkin.GetComponent<MeshFilter>();
                    handColliderBuilders = new HandColliderBuilder[2];
                    Matrix4x4[] leftRest = hands[0].GetBoneTransformations();
                    Matrix4x4[] rightRest =  hands[1].GetBoneTransformations();
                    handColliderBuilders[0] = new HandColliderBuilder(leftRest,leftHandSkin);
                    handColliderBuilders[1] = new HandColliderBuilder(rightRest, rightHandSkin);
                }   
                else{
                    Debug.Log("HandSkins are not found ");
                }
            }
            // update finger collider
            // runtimeFingerColliderUpdate();
            // joint distances
            leftContactDis = handColliderBuilders[0].GetJointContactLabel(IKFurthestDistance, true);
            rightContactDis = handColliderBuilders[1].GetJointContactLabel(IKFurthestDistance, true);
        }
    }
    private void InitObjectCollider(bool rebuild = false){
        if(replacedObject!=null){
            if(objectColliderBuilder==null || rebuild){
                objectColliderBuilder = new ObjectColliderBuilder(replacedObject);
            }
            // update 
            // runtimeObjectUpdate();
            // init local transformation for converting from/to certer
            centerLocal = replacedObject.GetComponent<ObjectManager>().GetCubeLocalTransformation(Axis.None);
            Matrix4x4 anotherTrans = Matrix4x4.TRS(anotherLocalOffset, Quaternion.Euler(anotherLocalRotation.x, anotherLocalRotation.y, anotherLocalRotation.z), Vector3.one);
            centerLocal = centerLocal*anotherTrans;
        }
    }
    private void InitWristObjectTransformation(){
        if(parser!=null && parser.trajData!=null){
            // init wrist and obejct trnasformation
            runtimeWristObjectTransformationUpdate((1f/GetFramerate())*(float)currentStep);
            // update hand and object collider
            runtimeObjectUpdate();
            runtimeFingerColliderUpdate();
        }
    }
    private void InitEnvSensor(){
        envSensor = new CuboidMap(envResolution);
        Matrix4x4 cuboidT = CuboidMapModule.GetCuboidTransformationFromWrist(hands[1], 1, Axis.None, envOffset);
        envSensor.Sense(cuboidT, envLayerMask, envSize);
    }
    private void InitProjectionVertices(){
        if(handColliderBuilders[0].handSkinObj != null){
            projectionVertexIndices = HandColliderModule.UpdateCurrentSkinVertexIndices(handColliderBuilders[0].handSkinObj.GetComponent<MeshFilter>().sharedMesh, 
                                                                                        projectionHandDirectionValue, 
                                                                                        projectionWristValue, 
                                                                                        projectionThumbDivisionValue, 
                                                                                        projectionThumbDirectionValue);
            Debug.Log("current mesh vertices numbers are " + projectionVertexIndices.Length);
        }
        else{
            Debug.Log("can not find handSkinObj from handColliderBuilder");
        }
    }
    private void InitNetwork(){
        // if(inputDim != ((SocketNetwork)NeuralNetwork).inputDim || outputDim  != ((SocketNetwork)NeuralNetwork).outputDim ){
		// 	inputDim = (int)((SocketNetwork)NeuralNetwork).inputDim;
		// 	outputDim = (int)((SocketNetwork)NeuralNetwork).outputDim;
        //     Debug.Log("Input/Output dim has been changed from " + inputDim + " , " + outputDim + " to " +  ((SocketNetwork)NeuralNetwork).inputDim + " , " + ((SocketNetwork)NeuralNetwork).outputDim);
		// }
        if(NeuralNetwork.Setup){
            int ManipInput = ((ManipNet)NeuralNetwork).inputDim;
            int ManipOutput = ((ManipNet)NeuralNetwork).denseRes.YDim;
            Debug.Log("Input/Output dim has been changed from " + inputDim + " , " + outputDim + " to " +  ManipInput+ " , " + ManipOutput);
            inputDim = ManipInput;
            outputDim = ManipOutput;
        }
        else
        {
            Debug.LogError("ManipNet is not setup");
        }
    }
    
    void LateUpdate() {
		Utility.SetFPS(Mathf.RoundToInt(GetFramerate()));
        if(replacedObject!=null){
            if(NeuralNetwork != null && NeuralNetwork.Setup) {
                // check current frame index
                if(currentStep >= startFrame+windowsize){
                    currentStep = startFrame;
                    // intial hand and object transformation
                    this.InitWristObjectTransformation();
                    // initialize Cuboid Sensor
                    this.InitEnvSensor();
                }
                float inputTimeStep = (1f/GetFramerate())*(float)currentStep;
                float outputTimeStep = (1f/GetFramerate())*(float)(currentStep+1);

                // predict left hand
                NeuralNetwork.ResetPivot(); FeedInput(inputTimeStep, false); NeuralNetwork.Predict(); NeuralNetwork.ResetPivot(); GetOutput(outputTimeStep, false);
                // predict right hand (right hand is called main hand)
                NeuralNetwork.ResetPivot(); FeedInput(inputTimeStep, true); NeuralNetwork.Predict(); NeuralNetwork.ResetPivot(); GetOutput(outputTimeStep, true);
                
                // play predicted motion for t+1
                PlayOutput(outputTimeStep);
                // for next round 
                currentStep += 1;
                // post proocess
                Postprocess();
                // draw the run time
                OnRenderObjectDerived();
            }
        }
        else{
            Debug.Log(" no replaced object ");
        }
		
    }

    // this is auto-regressive
	private void FeedInput(float timeStep, bool isMain){
        // mirror
        Axis currentMirrorAxis = isMain? Axis.None:mirrorAxis;
        int mainIndex = isMain? 1:0;
        int viceIndex = isMain? 0:1; 
        // update trajectory information
        trajectorySegments.UpdateSegments(timeStep, currentMirrorAxis, 
                                          trajectoryPastKeys, trajectoryFutureKeys, 
                                          trajectoryPastWindow, trajectoryFutureWindow, 
                                          1f/GetFramerate(), 
                                          trajectoryFilterSize, trajectoryFilterPower, 
                                          false);
		// root
        Matrix4x4[] Root = new Matrix4x4[2];
        Root = new Matrix4x4[2];
        Root[0] = hands[0].Bones[0].Transform.GetWorldMatrix().GetMirror(currentMirrorAxis);
        Root[1] = hands[1].Bones[0].Transform.GetWorldMatrix().GetMirror(currentMirrorAxis);
        {//feed in predicted main hand position
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                Matrix4x4 currentPosture = hands[mainIndex].Bones[iJoint].Transform.GetWorldMatrix().GetMirror(currentMirrorAxis);
                NeuralNetwork.Feed(currentPosture.GetPosition().GetRelativePositionTo(Root[mainIndex]));
            }
            // feed in special position of inference hands
            int nFingerTips = (int)fingerTipPositions.Length/2;
            for(int iJoint=0; iJoint<nFingerTips; iJoint++){
                Vector3 currentPosition = fingerTipPositions[mainIndex*nFingerTips+iJoint].GetMirror(currentMirrorAxis);
                NeuralNetwork.Feed(currentPosition.GetRelativePositionTo(Root[mainIndex]));
            }
            int nPalmColliders = (int)palmColliderStartPositions.Length/2;
            for(int iJoint=0; iJoint<nPalmColliders; iJoint++){
                Vector3 currentPosition = palmColliderStartPositions[mainIndex*nPalmColliders+iJoint].GetMirror(currentMirrorAxis);
                NeuralNetwork.Feed(currentPosition.GetRelativePositionTo(Root[mainIndex]));
            }
            for(int iJoint=0; iJoint<nPalmColliders; iJoint++){
                Vector3 currentPosition = palmColliderEndPositions[mainIndex*nPalmColliders+iJoint].GetMirror(currentMirrorAxis);
                NeuralNetwork.Feed(currentPosition.GetRelativePositionTo(Root[mainIndex]));
            }
            // feed in predicted rotations
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                Matrix4x4 currentPosture = hands[mainIndex].Bones[iJoint].Transform.GetWorldMatrix().GetMirror(currentMirrorAxis);
                NeuralNetwork.Feed(currentPosture.GetForward().GetRelativeDirectionTo(Root[mainIndex]));
                NeuralNetwork.Feed(currentPosture.GetUp().GetRelativeDirectionTo(Root[mainIndex]));
            }
        }

        // read the trajectories
        Matrix4x4[] viceSegment = trajectorySegments.GetVisualLeftWristSegment();
        Matrix4x4[] mainSegment = trajectorySegments.GetVisualRightWristSegment();
        Matrix4x4[] objectSegment = trajectorySegments.GetVisualObjectSegment();
        Vector3[] axisAngleV_RSegment = trajectorySegments.GetVisualObjectAxisAngleV_R();
        {// feed in trajectories
            for(int iSample=0; iSample<trajectorySegments.segmentLength; iSample++) {
                Vector3 relativeViceWrist = viceSegment[iSample].GetPosition().GetRelativePositionTo(Root[mainIndex]);
                float viceWristDistance = relativeViceWrist.magnitude;
                NeuralNetwork.Feed(viceWristDistance);
                NeuralNetwork.Feed(mainSegment[iSample].GetPosition().GetRelativePositionTo(Root[mainIndex]));
                NeuralNetwork.Feed(mainSegment[iSample].GetForward().GetRelativeDirectionTo(Root[mainIndex]));
                NeuralNetwork.Feed(mainSegment[iSample].GetUp().GetRelativeDirectionTo(Root[mainIndex]));
                
                NeuralNetwork.Feed(objectSegment[iSample].GetPosition().GetRelativePositionTo(Root[mainIndex]));
                NeuralNetwork.Feed(axisAngleV_RSegment[iSample]);
            }
        }

        {// feed in env
            {// re-place the object
                runtimeWristObjectTransformationUpdate(timeStep);
                runtimeObjectUpdate();
            }
            // if vice hand, need to mirror
            Vector3 offsetVector = Vector3.one.GetMirror(currentMirrorAxis);
            replacedObject.transform.parent.localScale = offsetVector;
            // get cuboid Trans
            Matrix4x4 cuboidT = Matrix4x4.identity;
            if(!isMain){
                cuboidT = CuboidMapModule.GetCuboidTransformationFromWrist(hands[0], 0, mirrorAxis, envOffset);
            }
            else{
                cuboidT = CuboidMapModule.GetCuboidTransformationFromWrist(hands[1], 1, Axis.None, envOffset);
            }
            envSensor.Sense(cuboidT, envLayerMask, envSize);
            for(int iVoxel=0; iVoxel<envSensor.Occupancies.Length; iVoxel++){ 
                NeuralNetwork.Feed(envSensor.Occupancies[iVoxel]);
            }
            replacedObject.transform.parent.localScale = Vector3.one;
        }
        
        {// distance representation
            int nDisatnces = futureDistanceLength+1;
            Matrix4x4[] runtimeRightWristTrans = new Matrix4x4[nDisatnces];
            Matrix4x4[] runtimeObjectTrans = new Matrix4x4[nDisatnces];
            Vector3 offsetVector = isMain? Vector3.one:Vector3.one.GetMirror(currentMirrorAxis);
            for(int iSample=0; iSample<nDisatnces; iSample++){
                // int indexTraj = (int)(handTimeSeries.Samples.Length/2)+iSample;
                int indexTraj =  (int) Mathf.Clamp(iSample*futureDistanceZoomOut + trajectorySegments.segmentLength/2, 0f, trajectorySegments.segmentLength-1);
                runtimeRightWristTrans[iSample] = mainSegment[indexTraj];
                //todo: note here, all the child transformations need to be mirrored
                runtimeObjectTrans[iSample] = objectSegment[indexTraj] * centerLocal.GetMirror(currentMirrorAxis).inverse; 
            }
            //   get runtime skin samples
            // get the current sampled points and directions
            SkinnedMeshRenderer skin = handColliderBuilders[mainIndex].handSkinObj.GetComponent<SkinnedMeshRenderer>();
            Mesh currentMesh = new Mesh();
            skin.BakeMesh(currentMesh);
            int nVertices = projectionVertexIndices.Length;
            Vector3[] vertexPositions = new Vector3[nVertices];
            Vector3[] vertexDirections = new Vector3[nVertices];
            for(int iV=0; iV<nVertices; iV++){
                vertexPositions[iV] = currentMesh.vertices[projectionVertexIndices[iV]].GetMirror(currentMirrorAxis);
                vertexDirections[iV] = currentMesh.normals[projectionVertexIndices[iV]].GetMirror(currentMirrorAxis);
            }
            
            // get the future distance and directions
            if(!isMain){
                /**if it is not the main hand, need to mirror the runtime object, 
                scale need to be mirrored, because UpdateCuboidMap and GetSignedDistanceValueViaKDTree is based on lossy scale
                can be improved...
                */
                objectColliderBuilder.GetObject().transform.parent.localScale = objectColliderBuilder.GetObject().transform.parent.localScale.GetMirror(currentMirrorAxis);
            }
            List<float[]> futureDistances = HandColliderModule.SenseFutureDistance(vertexPositions, vertexDirections, 
                                                                                   objectColliderBuilder,
                                                                                   runtimeRightWristTrans, runtimeObjectTrans, 
                                                                                   futureDistancePosThreshold, futureDistanceNegThreshold);
            {// for visualization/debug only
                ttposThreshold = futureDistancePosThreshold;
                ttPoints = new Vector3[nVertices];
                ttDirections = new Vector3[nVertices];
                ttDistance = new float[nVertices];
                for(int iV=0; iV<nVertices; iV++){
                    ttPoints[iV] = vertexPositions[iV];
                    ttDirections[iV] = vertexDirections[iV];
                    ttDistance[iV] = futureDistances[0][iV];
                }
            }

            // *************** calculateNN **************
            //   get runtime main hand position, if not the main hand need to mirror
            Vector3[] jointPositions = new Vector3[nJoint];
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                jointPositions[iJoint] = hands[mainIndex].Bones[iJoint].Transform.GetWorldMatrix().GetMirror(currentMirrorAxis).GetPosition();
            }
            //   get runtime main hand finger tip position, if not the main hand need to mirror
            int nFingerTips = (int)fingerTipPositions.Length/2;
            Vector3[] tipPositions = new Vector3[nFingerTips];
            for(int iJoint=0; iJoint<nFingerTips; iJoint++){
                tipPositions[iJoint] = fingerTipPositions[mainIndex*nFingerTips+iJoint].GetMirror(currentMirrorAxis);
            }
            // concate and get final joint positions in the world space
            jointPositions = ArrayExtensions.Concat(jointPositions, tipPositions);
            List<Vector3[]> nnFutureDirections = new List<Vector3[]>();
            List<float[]> nnFutureDistances = HandColliderModule.SenseNNFutureDistance(out nnFutureDirections, jointPositions, 
                                                                                       objectColliderBuilder, runtimeRightWristTrans, runtimeObjectTrans, 
                                                                                       futureDistancePosThreshold);
            // this is normalized NN normal directions
            List<Vector3> nnNormDirections = new List<Vector3>();
            nnNormDirections = HandColliderModule.SenseNNSurfaceNorms(jointPositions, nnFutureDirections[0], objectColliderBuilder);
            // *************** calculateNN **************

            // input current and future projection distance
            for(int iStep=0; iStep<nDisatnces; iStep++){
                for(int iSample = 0; iSample<nVertices; iSample++){
                    NeuralNetwork.Feed(futureDistances[iStep][iSample]);
                }
            }
            // input current and future NN distance
            for(int iStep=0; iStep<nDisatnces; iStep++){
                for(int iSample = 0; iSample<nJoint+nFingerTips; iSample++){
                    NeuralNetwork.Feed(nnFutureDistances[iStep][iSample]);
                }
            }
            // input norm directoin on the object surface at current frame only
            for(int iSample = 0; iSample<nJoint+nFingerTips; iSample++){
                NeuralNetwork.Feed(nnNormDirections[iSample].GetRelativeDirectionTo(Root[mainIndex]));
            }
            if(!isMain){
                // need to place back
                objectColliderBuilder.GetObject().transform.parent.localScale = objectColliderBuilder.GetObject().transform.parent.localScale.GetMirror(currentMirrorAxis);
            }    
        }
	}

    private void GetOutput(float timeStep, bool isMain){
        /// <summary>
        /// 1. read the network output
        /// 2. set mirror and get the root at output time step     
        /// 3. get the global finger transformations
        /// 4. get the predicted contact distance
        /// <summary>

        // read the network output
        Vector3[] jointPositions = new Vector3[nJoint];
        Vector3[] jointForwards = new Vector3[nJoint];
        Vector3[] jointUps = new Vector3[nJoint];
        {   
            // read the joint position
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                jointPositions[iJoint] = NeuralNetwork.ReadVector3();
            }
            // read special position but do not use, because will reconstruct from the colliders
            int nFingerTips = (int)fingerTipPositions.Length/2;
            for(int iJoint=0; iJoint<nFingerTips; iJoint++){
                Vector3 position = NeuralNetwork.ReadVector3();
            }
            int nPalmColliders = (int)palmColliderStartPositions.Length/2;
            for(int iJoint=0; iJoint<nPalmColliders; iJoint++){
                Vector3 position = NeuralNetwork.ReadVector3();
            }
            for(int iJoint=0; iJoint<nPalmColliders; iJoint++){
                Vector3 position = NeuralNetwork.ReadVector3();
            }
            // read joint rotation
            for(int iJoint=0; iJoint<nJoint; iJoint++){
                jointForwards[iJoint] = NeuralNetwork.ReadVector3();
                jointUps[iJoint] = NeuralNetwork.ReadVector3();
            }
        }

        {// deal with the relative problem and get the world transformation
            // set mirror and get the root at output time step
            Axis currentMirrorAxis = isMain? Axis.None:mirrorAxis;
            Matrix4x4 outRoot = Matrix4x4.identity;
            outRoot = parser.trajData.GetVisualRightWristTrans(timeStep, currentMirrorAxis, false);
            // get the world transformation
            if(predictedPositions.Length==0 || predictedRotations.Length==0){
                predictedPositions = new Vector3[nHand*nJoint];
                predictedRotations = new Quaternion[nHand*nJoint];
            }
            // right hand simple relative, left hand relative to mirrored left then mirror back
            if(isMain){
                for(int iJoint=0; iJoint<nJoint; iJoint++){
                    predictedPositions[nJoint+iJoint] = jointPositions[iJoint].GetRelativePositionFrom(outRoot);
                    jointForwards[iJoint] = jointForwards[iJoint].GetRelativeDirectionFrom(outRoot);
                    jointUps[iJoint] = jointUps[iJoint].GetRelativeDirectionFrom(outRoot);
                    predictedRotations[nJoint+iJoint] = Quaternion.LookRotation(jointForwards[iJoint], jointUps[iJoint]);
                }
            }
            else{
                for(int iJoint=0; iJoint<nJoint; iJoint++){
                    predictedPositions[iJoint] = jointPositions[iJoint].GetRelativePositionFrom(outRoot);
                    jointForwards[iJoint] = jointForwards[iJoint].GetRelativeDirectionFrom(outRoot);
                    jointUps[iJoint] = jointUps[iJoint].GetRelativeDirectionFrom(outRoot);
                    predictedRotations[iJoint] = Quaternion.LookRotation(jointForwards[iJoint], jointUps[iJoint]);
                    Matrix4x4 predictedTrans = Matrix4x4.TRS(predictedPositions[iJoint], predictedRotations[iJoint], Vector3.one);
                    predictedTrans = predictedTrans.GetMirror(mirrorAxis);
                    predictedPositions[iJoint] = predictedTrans.GetPosition();
                    predictedRotations[iJoint] = predictedTrans.GetRotation();
                }
            }
        }
        {// read the main hand distance output from network
            if(isMain){
                for(int iJoint=0; iJoint<rightContactDis.Length; iJoint++){
                    rightContactDis[iJoint] = Mathf.Clamp(NeuralNetwork.Read(), 0f, 1f);
                }
            }
            else{
                for(int iJoint=0; iJoint<leftContactDis.Length; iJoint++){
                    leftContactDis[iJoint] = Mathf.Clamp(NeuralNetwork.Read(), 0f, 1f);
                }
            }
        }
	}

    private void PlayOutput(float timeStep){
        /// <summary>
        /// 1. update the wrist and object transformation
        /// 2. update the finger transformation
        /// 3. update handCollider and Object Collider
        /// </summary>
        
        // update the wrist and object transformation
        runtimeWristObjectTransformationUpdate(timeStep);
        // update the finger transformation
        for(int iHand=0; iHand<nHand; iHand++){
            for(int iJoint=1; iJoint<nJoint; iJoint++){
                hands[iHand].Bones[iJoint].Transform.position = predictedPositions[iHand*nJoint+iJoint];
                hands[iHand].Bones[iJoint].Transform.rotation = predictedRotations[iHand*nJoint+iJoint];
            }
        }
        // update the hand collider and object collider
        runtimeObjectUpdate();
        runtimeFingerColliderUpdate();
    }

    private void runtimeWristObjectTransformationUpdate(float timeStep){
        if(parser!=null && parser.trajData!=null){
            // todo different from CSV
            Matrix4x4 leftTrans =  parser.trajData.GetVisualLeftWristTrans(timeStep, Axis.None, false);
            Matrix4x4 rightTrans = parser.trajData.GetVisualRightWristTrans(timeStep, Axis.None, false);
            Matrix4x4 objTrans = parser.trajData.GetVisualObjectTrans(timeStep, Axis.None, false);
            // todo different from CSV

            hands[0].Bones[0].Transform.position = leftTrans.GetPosition();
            hands[0].Bones[0].Transform.rotation = leftTrans.GetRotation();
            hands[1].Bones[0].Transform.position = rightTrans.GetPosition();
            hands[1].Bones[0].Transform.rotation = rightTrans.GetRotation();

            objTrans = objTrans * centerLocal.inverse;
            replacedObject.transform.position = objTrans.GetPosition();
            replacedObject.transform.rotation = objTrans.GetRotation();

            // update local for adjusting during runtime
            centerLocal = replacedObject.GetComponent<ObjectManager>().GetCubeLocalTransformation(Axis.None);
            Matrix4x4 anotherTrans = Matrix4x4.TRS(anotherLocalOffset, Quaternion.Euler(anotherLocalRotation.x, anotherLocalRotation.y, anotherLocalRotation.z), Vector3.one);
            centerLocal = centerLocal*anotherTrans;
        }
    }
	private void runtimeObjectUpdate(){
        if(objectColliderBuilder!=null){
            objectColliderBuilder.UpdateCuboidMap();
        }
	}
    private void runtimeFingerColliderUpdate(){
        handColliderBuilders[0].UpdateJointColliderWorldT(hands[0], true);
        handColliderBuilders[1].UpdateJointColliderWorldT(hands[1], true);
        fingerTipPositions = ArrayExtensions.Concat(
                        handColliderBuilders[0].GetFingerTipPositions(), 
                        handColliderBuilders[1].GetFingerTipPositions());
        palmColliderStartPositions = ArrayExtensions.Concat(
                            handColliderBuilders[0].GetPalmCapsuleStartPositions(), 
                            handColliderBuilders[1].GetPalmCapsuleStartPositions());
        palmColliderEndPositions = ArrayExtensions.Concat(
                            handColliderBuilders[0].GetPalmCapsuleEndPositions(), 
                            handColliderBuilders[1].GetPalmCapsuleEndPositions());
    }

	protected override void Postprocess() {
        if(handColliderBuilders!=null){
            if(IK){
                MeshCollider meshCollider = replacedObject.GetComponentInChildren<MeshCollider>();
                CheckSDF checkSDF = replacedObject.GetComponent<CheckSDF>();
                Matrix4x4[] leftRestPose = leftIK==null? GetRestTranses(0) : null;
                Matrix4x4[] rightRestPose = rightIK==null? GetRestTranses(1) : null;
                leftIK = HandIK.PostJointDistanceIK(hands[0], handColliderBuilders[0], checkSDF, 
                                                    leftContactDis, IKFurthestDistance, IKDiscountThreshold,
                                                    leftIK,
                                                    out contactPointsLeft,
                                                    false,
                                                    leftRestPose);
                rightIK = HandIK.PostJointDistanceIK(hands[1],handColliderBuilders[1], checkSDF, 
                                                     rightContactDis, IKFurthestDistance, IKDiscountThreshold,
                                                     rightIK,
                                                     out contactPointsRight,
                                                     true,
                                                     rightRestPose);
                runtimeFingerColliderUpdate();

                { 
                    // this is optional for solving further penetrations
                    leftIK = HandIK.PostPeneIK(hands[0], handColliderBuilders[0], 
                                                objectColliderBuilder.GetMeshColliders(),
                                                IKDiscountThreshold,
                                                leftIK,
                                                false,
                                                leftRestPose);
                    rightIK = HandIK.PostPeneIK(hands[1], handColliderBuilders[1], 
                                                objectColliderBuilder.GetMeshColliders(),
                                                IKDiscountThreshold,
                                                rightIK,
                                                true,
                                                rightRestPose);
                    runtimeFingerColliderUpdate();
                }
            }
        }
    }
	
	protected override void OnRenderObjectDerived() {
        if(handColliderBuilders!=null && drawHandColliders){
            // handColliderBuilders[0].UpdateJointColliderWorldT(hands[0], true);
            handColliderBuilders[0].DrawJointColliders(UltiDraw.Green.Transparent(0.5f), UltiDraw.Red.Transparent(0.5f), leftContactDis, IKDiscountThreshold);

            // handColliderBuilders[1].UpdateJointColliderWorldT(hands[1], true);
            handColliderBuilders[1].DrawJointColliders(UltiDraw.Green.Transparent(0.5f), UltiDraw.Red.Transparent(0.5f), rightContactDis, IKDiscountThreshold);
        }    
        if(objectColliderBuilder!=null && drawObjectColliders){
            objectColliderBuilder.DrawContact(UltiDraw.Green.Transparent(0.5f), UltiDraw.Red.Transparent(0.5f), IKFurthestDistance);
        }
        if(drawContact){
            UltiDraw.Begin();
            for(int iContact=0; iContact<contactPointsLeft.Count; iContact++){
                if(iContact%2==0){
                    UltiDraw.DrawSphere(contactPointsLeft[iContact], Quaternion.identity, 0.05f, UltiDraw.Black);
                }
                else{
                    UltiDraw.DrawSphere(contactPointsLeft[iContact], Quaternion.identity, 0.05f, UltiDraw.Blue);
                }
            }
            for(int iContact=0; iContact<contactPointsRight.Count; iContact++){
                if(iContact%2==0){
                    UltiDraw.DrawSphere(contactPointsRight[iContact], Quaternion.identity, 0.05f, UltiDraw.Black);
                }
                else{
                    UltiDraw.DrawSphere(contactPointsRight[iContact], Quaternion.identity, 0.05f, UltiDraw.Blue);
                }
            }
            UltiDraw.End();
        }

        if(drawProjectionRays){
            UltiDraw.Begin();
            for(int i=0; i<ttPoints.Length; i++){
                if(ttDistance[i]<ttposThreshold){
                    UltiDraw.DrawArrow(
                        ttPoints[i],
                        ttPoints[i] + ttDirections[i] * ttDistance[i],
                        0.75f, 0.0075f, 0.05f, UltiDraw.DarkRed
                    );
                }
                else{
                    UltiDraw.DrawArrow(
                        ttPoints[i],
                        ttPoints[i] + ttDirections[i] * ttposThreshold,
                        0.75f, 0.0075f, 0.05f, UltiDraw.DarkGrey
                    );
                }
            }
            UltiDraw.End();
        }

        if(drawSpecialPositions){
            UltiDraw.Begin();
            for(int iP=0; iP<fingerTipPositions.Length; iP++){
                UltiDraw.DrawSphere(fingerTipPositions[iP], Quaternion.identity, 0.05f, UltiDraw.Yellow);
            }
            for(int iP=0; iP<palmColliderStartPositions.Length; iP++){
                UltiDraw.DrawSphere(palmColliderStartPositions[iP], Quaternion.identity, 0.05f, UltiDraw.Yellow);
            }
            for(int iP=0; iP<palmColliderEndPositions.Length; iP++){
                UltiDraw.DrawSphere(palmColliderEndPositions[iP], Quaternion.identity, 0.05f, UltiDraw.Yellow);
            }
            UltiDraw.End();
        }
    
        if(drawEnv){
            if(envSensor!=null){
                envSensor.Draw(UltiDraw.Cyan);
            }
        }
    }


    private void InitDemo1(){
        demo = DEMO.Demo1;
        // change trajectory and object
        replacedObject.gameObject.SetActive(false);
        GameObject parentObject = GameObject.Find("Objects");
        replacedObject = parentObject.transform.Find("cyl_med").gameObject;
        replacedObject.gameObject.SetActive(true);

        txtPath = rootPath +  "Trajectories/demo1.txt";
        
        this.ReloadRestPoses();
        this.InitCSVTrajectories();
        this.InitObjectCollider(true);
        this.InitWristObjectTransformation();
        this.InitEnvSensor();

        startFrame = 30;
        windowsize = 1000;
        currentStep = startFrame;

        // camera
        GameObject camera = GameObject.Find("Camera");
        camera.transform.position = new Vector3(-8.5f, 6.5f, 4.5f);
        camera.transform.rotation = Quaternion.Euler(45f, 90f, 0f);

        // network
        if(((ManipNet)NeuralNetwork).Folder !=  rootPath + "ManipNetBIN"){
            ((ManipNet)NeuralNetwork).Folder =  rootPath + "ManipNetBIN";
            ((ManipNet)NeuralNetwork).ReSetUp();
        }
    }

    private void InitDemo2(string objectName){
        demo = DEMO.Demo2;
        // change trajectory and object
        replacedObject.gameObject.SetActive(false);
        GameObject parentObject = GameObject.Find("Objects");
        replacedObject = parentObject.transform.Find(objectName).gameObject;
        replacedObject.gameObject.SetActive(true);
        txtPath = rootPath +  "Trajectories/demo2.txt";
        this.ReloadRestPoses();
        this.InitCSVTrajectories();
        this.InitObjectCollider(true);
        this.InitWristObjectTransformation();
        this.InitEnvSensor();

        startFrame = 40;
        windowsize = 300;
        currentStep = startFrame;   

        // camera
        GameObject camera = GameObject.Find("Camera");
        camera.transform.position = new Vector3(-6.5f, 6f, 4.5f);
        camera.transform.rotation = Quaternion.Euler(45f, 90f, 0f);

        // network
        if(((ManipNet)NeuralNetwork).Folder !=  rootPath + "ManipNetBIN"){
            ((ManipNet)NeuralNetwork).Folder =  rootPath + "ManipNetBIN";
            ((ManipNet)NeuralNetwork).ReSetUp();
        }
    }

    private GUIStyle GetButtonStyle() {
		if(ButtonStyle == null) {
			ButtonStyle = new GUIStyle(GUI.skin.button);
			ButtonStyle.font = (Font)Resources.Load("Fonts/Coolvetica");
			ButtonStyle.normal.textColor = Color.white;
			ButtonStyle.alignment = TextAnchor.MiddleCenter;
		}
		return ButtonStyle;
	}

    protected override void OnGUIDerived() {
		GetButtonStyle().fontSize = Mathf.RoundToInt(0.01f * Screen.width);
        {// Demos
            {// DEMO1
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = demo == DEMO.Demo1? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.85f, 0.05f, 0.13f, 0.05f), "Demo1", GetButtonStyle())) {
                    Debug.Log("Demo1");
                    InitDemo1();
                }
            }
            
            {// DEMO2
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = demo == DEMO.Demo2? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.85f, 0.2f, 0.13f, 0.05f), "Demo2", GetButtonStyle())) {
                    Debug.Log("Demo2");
                    demoObj = DEMOOBJ.Sphere;
                    InitDemo2("sphere");
                }
                
                if(demo == DEMO.Demo2){
                    //  {Sphere, Cylinder, Cube, TriPrism, Hemi, Tetra, Bunny}
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.Sphere? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.25f, 0.1f, 0.04f), "Sphere", GetButtonStyle())) {
                            demoObj = DEMOOBJ.Sphere;
                            InitDemo2("sphere");
                        }
                    }
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.Cylinder? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.3f, 0.1f, 0.04f), "Cylinder", GetButtonStyle())) {
                            demoObj = DEMOOBJ.Cylinder;
                            InitDemo2("cyl_med");
                        }
                    }
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.Cube? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.35f, 0.1f, 0.04f), "Cube", GetButtonStyle())) {
                            demoObj = DEMOOBJ.Cube;
                            InitDemo2("cube_med");
                        }
                    }
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.TriPrism? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.4f, 0.1f, 0.04f), "TriPrism", GetButtonStyle())) {
                            demoObj = DEMOOBJ.TriPrism;
                            InitDemo2("tri_prism_large");
                        }
                    }  
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.Hemi? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.45f, 0.1f, 0.04f), "Hemi", GetButtonStyle())) {
                            demoObj = DEMOOBJ.Hemi;
                            InitDemo2("hemi_large");
                        }
                    }
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.Tetra? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.5f, 0.1f, 0.04f), "Tetra", GetButtonStyle())) {
                            demoObj = DEMOOBJ.Tetra;
                            InitDemo2("tetra_large");
                        }
                    }
                    {
                        GUI.backgroundColor = demoObj == DEMOOBJ.Bunny? UltiDraw.Black:UltiDraw.Cyan;
                        if(GUI.Button(Utility.GetGUIRect(0.85f, 0.55f, 0.1f, 0.04f), "Bunny", GetButtonStyle())) {
                            demoObj = DEMOOBJ.Bunny;
                            InitDemo2("bunnyTest");
                        }
                    }
                  
                }
            }
        }

        { // Visualizations
            {// draw hand collider
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = drawHandColliders? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.05f, 0.05f, 0.13f, 0.05f), "Draw Hand Colliders", GetButtonStyle())) {
                    drawHandColliders = !drawHandColliders;
                }
            }    
            {// draw object collider
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = drawObjectColliders? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.05f, 0.12f, 0.13f, 0.05f), "Draw Object Colliders", GetButtonStyle())) {
                    drawObjectColliders = !drawObjectColliders;
                }
            }

            {// draw contact
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = drawContact? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.05f, 0.19f, 0.13f, 0.05f), "Draw Tip Nearest Pairs", GetButtonStyle())) {
                    drawContact = !drawContact;
                }
            }

            {// draw ambient sensor
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = drawEnv? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.05f, 0.26f, 0.13f, 0.05f), "Draw Ambient", GetButtonStyle())) {
                    drawEnv = !drawEnv;
                }
            }

            {// draw proximity
                GUI.color = UltiDraw.White;
                GUI.backgroundColor = drawProjectionRays? UltiDraw.Black:UltiDraw.Cyan;
                if(GUI.Button(Utility.GetGUIRect(0.05f, 0.33f, 0.13f, 0.05f), "Draw Proximity", GetButtonStyle())) {
                    drawProjectionRays = !drawProjectionRays;
                }
            }
           
            

        }
    }


    void OnApplicationQuit()
    {
        ReloadRestPoses();
    }
	protected override void Feed() {}
	protected override void Read() {}
}